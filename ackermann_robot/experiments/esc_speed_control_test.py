#!/usr/bin/env python3
"""Closed-loop forward/reverse ESC speed tracking and tuning experiment."""
import argparse, csv, math, os, statistics, tempfile, time
import rclpy
import yaml
from ackermann_robot.experiments.esc_forward_calibrate import (
    NEUTRAL, US_PER_TICK, Experiment, neutral, open_bus, write_tick)


def interpolate(points, speed):
    points = sorted(points)
    if speed <= points[0][0]: return points[0][1]
    if speed >= points[-1][0]: return points[-1][1]
    for (s0,p0),(s1,p1) in zip(points, points[1:]):
        if s0 <= speed <= s1:
            return p0+(p1-p0)*(speed-s0)/(s1-s0)


def local_gain(points, speed):
    """Derive conservative P gain from the local pulse/speed slope."""
    points=sorted(points)
    pair=(points[0],points[1]) if speed<=points[0][0] else (points[-2],points[-1])
    for a,b in zip(points,points[1:]):
      if a[0]<=speed<=b[0]: pair=a,b;break
    slope=abs((pair[1][1]-pair[0][1])/(pair[1][0]-pair[0][0]))
    return max(15.0,min(35.0,.5*slope))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm',action='store_true')
    p.add_argument('--hold-seconds',type=float,default=8.0)
    p.add_argument('--kp',type=float,default=0.0,help='override derived P gain; 0=auto')
    p.add_argument('--ki',type=float,default=0.0,help='unused in proportional-only experiment')
    p.add_argument('--max-speed',type=float,default=.40)
    p.add_argument('--max-distance',type=float,default=3.00)
    p.add_argument('--stop-range',type=float,default=.20)
    p.add_argument('--output',default='esc_closed_loop.csv')
    p.add_argument('--startup-output',default='esc_startup_ramps.csv')
    p.add_argument('--model',default='~/.robot/esc_model.yaml')
    args,ros_args=p.parse_known_args()
    if not args.arm:p.error('refusing hardware output without --arm')
    if not 2<=args.hold_seconds<=10:p.error('--hold-seconds must be 2..10')
    args.breakaway_speed=.04  # replaced by measured neutral noise after bringup
    maps={
      # Refit from fresh-odom P-only run: 1415.845 us -> 0.269 m/s and
      # 1414.354 us -> 0.321 m/s. Start conservatively near neutral.
      'forward':[(.20,1419.0),(.24,1417.4),(.25,1417.0),(.28,1416.0),(.31,1415.0)],
      'reverse':[(.20,1592.0),(.25,1593.5),(.28,1594.2),(.30,1594.8)]}
    # Milestone-1 safe learning envelope. Repeat every target so the second
    # pass validates the pulse and integral learned on the first pass.
    # Surface-transition adaptability test: place the carpet boundary near
    # the midpoint of each ~1 m segment so both transition directions repeat.
    sequence=[.25,-.25,.25,-.25]
    model_path=os.path.expanduser(args.model)
    model={}
    if os.path.exists(model_path):
      with open(model_path) as f:model=yaml.safe_load(f) or {}
      print('Loaded persistent model:',model_path)
    rclpy.init(args=ros_args);bus=None
    with open(args.output,'w',newline='') as output, open(args.startup_output,'w',newline='') as startup_output:
      w=csv.writer(output);w.writerow(['segment','elapsed_s','target_mps','measured_mps',
        'error_mps','feedforward_us','command_us','tick','fresh_odom','tf_stamp',
        'x_m','y_m','yaw_rad','distance_m','overspeed_supervisor'])
      sw=csv.writer(startup_output);sw.writerow(['segment','direction','phase','step','elapsed_s',
        'pulse_us','result','signed_speed_mps','distance_m','next_seed_us'])
      node=Experiment(args,writer=None);started=time.monotonic()
      try:
        bus=open_bus();neutral(bus);node.spin_for(3)
        neutral_samples=node.velocity_history[-30:]
        if len(neutral_samples)<10:
          raise RuntimeError('insufficient fresh odometry for noise calibration')
        noise_center=statistics.median(neutral_samples)
        noise_mad=statistics.median(abs(v-noise_center) for v in neutral_samples)
        noise_sigma=max(.001,1.4826*noise_mad)
        movement_threshold=max(.020,min(.060,4.0*noise_sigma))
        recovery_threshold=min(.100,movement_threshold+max(.015,2.0*noise_sigma))
        overspeed_margin=max(.030,3.0*noise_sigma)
        overspeed_span=max(.070,6.0*noise_sigma)
        args.breakaway_speed=movement_threshold
        print(f'ODOM NOISE: sigma={noise_sigma:.4f} m/s, movement={movement_threshold:.4f}, '
              f'recovery={recovery_threshold:.4f}')
        baseline={float(k):float(v) for k,v in model.get('feedforward_us',{}).items()}
        session_trim={target:0.0 for target in sequence}
        # Start from persisted history when available. Version-1 models have
        # only the last seed and migrate naturally on the first new event.
        stored_seed=model.get('breakaway_seed_us',{})
        kick_seed={'forward':float(stored_seed.get('forward',1410.0)),
                   'reverse':float(stored_seed.get('reverse',1595.0))}
        stored_history=model.get('breakaway_history_us',{})
        breakaway_history={
          'forward':[float(x) for x in stored_history.get('forward',[])][-20:],
          'reverse':[float(x) for x in stored_history.get('reverse',[])][-20:]}
        def save_model():
          os.makedirs(os.path.dirname(model_path),exist_ok=True)
          payload={'version':2,'updated_unix':time.time(),
            'breakaway_seed_us':kick_seed,
            'breakaway_history_us':breakaway_history,
            'breakaway_typical_us':{
              d:(statistics.median(v) if v else None)
              for d,v in breakaway_history.items()},
            'breakaway_high_load_us':{
              'forward':(min(breakaway_history['forward']) if breakaway_history['forward'] else None),
              'reverse':(max(breakaway_history['reverse']) if breakaway_history['reverse'] else None)},
            'feedforward_us':{str(k):v for k,v in sorted(baseline.items())},
            'last_session_trim_us':{str(k):v for k,v in sorted(session_trim.items())}}
          payload['derived']={'odom_noise_sigma_mps':noise_sigma,
            'movement_threshold_mps':movement_threshold,
            'recovery_threshold_mps':recovery_threshold,
            'pca_us_per_tick':US_PER_TICK}
          fd,tmp=tempfile.mkstemp(prefix='.esc_model.',dir=os.path.dirname(model_path))
          try:
            with os.fdopen(fd,'w') as f:yaml.safe_dump(payload,f,sort_keys=True)
            os.replace(tmp,model_path)
          finally:
            if os.path.exists(tmp):os.unlink(tmp)
        def run_breakaway_ramp(segment,direction,sign,phase,max_duration=None,start_pulse=None):
          kick=kick_seed[direction] if start_pulse is None else start_pulse
          ramp_started=time.monotonic();ramp_step=0
          limit=1398.0 if sign>0 else 1612.0
          result='continue'
          while (kick>=limit if sign>0 else kick<=limit):
            if max_duration and time.monotonic()-ramp_started>=max_duration:
              raise RuntimeError(f'segment {segment} {phase} timed out')
            ramp_step+=1
            result=node.apply(bus,kick,.20,sign)
            if result=='breakaway':
              history=breakaway_history[direction]
              history.append(kick);del history[:-20]
              typical=statistics.median(history)
              kick_seed[direction]=(typical+1.0 if sign>0 else typical-1.0)
              print(f'segment {segment} {direction} {phase} breakaway at {kick:.1f} us; '
                    f'typical {typical:.1f}, next seed {kick_seed[direction]:.1f} us')
            sw.writerow([segment,direction,phase,ramp_step,
              f'{time.monotonic()-ramp_started:.3f}',f'{kick:.2f}',result,
              f'{node.signed_speed:.6f}',f'{node.distance:.6f}',
              f'{kick_seed[direction]:.2f}']);startup_output.flush()
            if result=='breakaway':save_model();return kick
            if result!='continue':
              raise RuntimeError(f'segment {segment} {phase} safety: {result}')
            if phase=='recovery':
              magnitude=US_PER_TICK if ramp_step<=4 else 2.0*US_PER_TICK
            else:
              magnitude=(.5*US_PER_TICK if ramp_step<=4 else
                         (US_PER_TICK if ramp_step<=8 else 2.0*US_PER_TICK))
            kick+=-magnitude if sign>0 else magnitude
          raise RuntimeError(f'segment {segment} {phase} found no breakaway')
        print('FRESH-ODOM P-ONLY: continuous output, updates only at MOLA 10 Hz')
        print('CLOSED-LOOP: alternating targets',sequence)
        for segment,target in enumerate(sequence,1):
          direction='forward' if target>0 else 'reverse';sign=1 if target>0 else -1
          node.direction=direction;node.origin=node.pose;node.signed_speed=node.speed=0.
          neutral(bus);node.spin_for(1.5)
          kick=run_breakaway_ramp(segment,direction,sign,'startup')
          base_ff=baseline.get(target,interpolate(maps[direction],abs(target)))
          ff=base_ff+session_trim[target]
          base_gain=args.kp if args.kp>0 else local_gain(maps[direction],abs(target))
          command=ff;error=target-node.signed_speed
          dither=0.;last=time.monotonic();end=last+args.hold_seconds
          settled=[];last_velocity=node.signed_speed;accel_filtered=0.0
          last_odom_seq=node.odom_seq
          overspeed_supervisor=False
          low_speed_samples=0;recovery_count=0;recovery_active=False
          recovery_samples=0;recovery_step=0
          post_recovery_guard=False;guard_samples=0;control_samples=0
          stall_sample_limit=3;recovery_sample_limit=22;guard_sample_limit=6
          recovery_increment=2.5*US_PER_TICK
          traction_success_threshold=max(recovery_threshold,.5*abs(target))
          add_traction_step=2.5*US_PER_TICK
          shed_traction_step=5.0*US_PER_TICK
          previous_fresh_speed=sign*node.signed_speed
          while rclpy.ok() and time.monotonic()<end:
            now=time.monotonic();dt=now-last;last=now
            rclpy.spin_once(node,timeout_sec=.02)
            fresh=node.odom_seq!=last_odom_seq
            if fresh:
              last_odom_seq=node.odom_seq
              control_samples+=1
              error=target-node.signed_speed
              speed_in_direction=sign*node.signed_speed
              fresh_speed_delta=speed_in_direction-previous_fresh_speed
              previous_fresh_speed=speed_in_direction
              if control_samples>5 and speed_in_direction<movement_threshold:
                low_speed_samples+=1
              elif speed_in_direction>recovery_threshold:
                low_speed_samples=0
              if recovery_active:
                recovery_samples+=1
                if speed_in_direction>traction_success_threshold:
                  sw.writerow([segment,direction,'traction_recovery',recovery_step,
                    f'{recovery_samples/10.0:.3f}',f'{command:.2f}','recovered',
                    f'{node.signed_speed:.6f}',f'{node.distance:.6f}',
                    f'{kick_seed[direction]:.2f}']);startup_output.flush()
                  recovery_active=False;recovery_count+=1;low_speed_samples=0
                  post_recovery_guard=True;guard_samples=0
                  command=1425.0 if sign>0 else 1575.0
                elif recovery_samples>=recovery_sample_limit:
                  raise RuntimeError(f'segment {segment}: traction recovery timed out')
                else:
                  recovery_step+=1
                  command+=-recovery_increment if sign>0 else recovery_increment
                  command=(max(1380.0,command) if sign>0 else min(1616.0,command))
                  sw.writerow([segment,direction,'traction_recovery',recovery_step,
                    f'{recovery_samples/10.0:.3f}',f'{command:.2f}','continue',
                    f'{node.signed_speed:.6f}',f'{node.distance:.6f}',
                    f'{kick_seed[direction]:.2f}']);startup_output.flush()
              elif low_speed_samples>=stall_sample_limit:
                if recovery_count>=2:
                  raise RuntimeError(f'segment {segment}: third stall after two recoveries')
                recovery_active=True;recovery_samples=0;recovery_step=0
                end=max(end,now+recovery_sample_limit*.12)
                print(f'segment {segment}: continuous traction recovery from {command:.1f} us')
              if recovery_active:
                pass  # hold/increase the continuous recovery command above
              elif post_recovery_guard:
                guard_samples+=1
                if ((speed_in_direction<=abs(target)+.02 and fresh_speed_delta<=0.0)
                    or guard_samples>=guard_sample_limit):
                  post_recovery_guard=False
                else:
                  command=1425.0 if sign>0 else 1575.0
                  overspeed_supervisor=True
              else:
                # Continuous gain scheduling replaces the old hard switch.
                # Gain rises from 20 to 60 as overspeed grows from +.03 to
                # +.10 m/s, producing proportionate braking authority.
                overspeed=max(0.0,speed_in_direction-(abs(target)+overspeed_margin))
                gain=base_gain+min(2.0*base_gain,
                                   2.0*base_gain*overspeed/overspeed_span)
                desired=ff-gain*error
                desired=(max(1408.,min(1425.,desired)) if sign>0
                         else max(1575.,min(1596.,desired)))
                # Pulse slew is asymmetric: shed traction faster (6 us per
                # fresh sample) than adding it (3 us). Emergency cutoff is
                # independent and remains immediate.
                delta=desired-command
                max_step=shed_traction_step if sign*delta>0 else add_traction_step
                command+=max(-max_step,min(max_step,delta))
                overspeed_supervisor=overspeed>0.0
            ideal=command/US_PER_TICK+dither;tick=round(ideal);dither=ideal-tick
            write_tick(bus,tick)
            if dt>0:
              acceleration=(node.signed_speed-last_velocity)/dt
              accel_filtered=.85*accel_filtered+.15*acceleration
            last_velocity=node.signed_speed
            okay,reason=node.safe()
            pose=node.pose or (math.nan,math.nan,math.nan,math.nan)
            w.writerow([segment,f'{now-started:.4f}',target,f'{node.signed_speed:.6f}',
              f'{error:.6f}',f'{ff:.3f}',f'{command:.3f}',tick,int(fresh),
              f'{pose[0]:.9f}',f'{pose[1]:.6f}',f'{pose[2]:.6f}',f'{pose[3]:.6f}',
              f'{node.distance:.6f}',int(overspeed_supervisor)])
            if (now>end-args.hold_seconds/2 and abs(accel_filtered)<.08
                and abs(error)<.015
                and not recovery_active and not overspeed_supervisor):
              settled.append((command,node.signed_speed))
            if not okay:raise RuntimeError(f'segment {segment}: {reason}')
          neutral(bus);output.flush()
          if settled:
            avg_p=sum(x[0] for x in settled)/len(settled)
            avg_v=sum(x[1] for x in settled)/len(settled)
            # Conservative online update: one surface patch must not replace
            # the whole feed-forward estimate used on the validation pass.
            observed_trim=max(-3.0,min(3.0,avg_p-base_ff))
            session_trim[target]=.8*session_trim[target]+.2*observed_trim
          else:
            avg_p,avg_v=command,node.signed_speed
          print(f'{segment}/{len(sequence)} target {target:+.2f}: '
                f'avg {avg_v:+.3f} m/s, session trim {session_trim[target]:+.2f} us')
          save_model()
          node.spin_for(1.2)
      except (KeyboardInterrupt,RuntimeError) as e:print('STOP:',e)
      finally:
        if bus:
          for _ in range(5):neutral(bus);time.sleep(.02)
          bus.close()
        node.destroy_node();rclpy.shutdown();print('ESC neutral. Log:',args.output)


if __name__=='__main__':main()
