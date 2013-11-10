#!/usr/bin/env python

print 'step, time, velocity, halfstep, to_step'
step = 0.0
# ms
time = 0.0
# Minimum velocity (steps a second)
velocity = 10.0

'''
Know that half width of 2000 results in 370 steps / second
'''

to_step = 100
while True:
    step += 1
    
    if to_step * 13.0 <= velocity:
        velocity -= 13.0
    else:  
        velocity += 13.0
    
    if velocity < 10:
        velocity = 10
    if velocity > 370:
        velocity = 370
    
    # 1 step / (steps/sec) = sec, sec * 1000 = ms
    time += 1.0 / velocity * 1000
    
    halfstep = 740000 / velocity
    print '%d, %d, %d, %d, %d' % (step, time, velocity, halfstep, to_step)

    #if time >= 250:
    #    break
    if step == 100:
        break
    to_step -= 1
    
