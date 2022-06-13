from time import time
from ir_sim.env import env_base
from potential_fields import potential_fields
from pathlib import Path

animation = True # whether generate the animation
image_path = Path(__file__).parent / 'image'  # image and animation path
gif_path = Path(__file__).parent / 'gif'

env = env_base(world_name = 'question3.yaml')
pf = potential_fields()
flag = False
for i in range(1000):

    if animation:
        env.save_fig(image_path, i) 


    ## please complete this part to solve question3 based on the force defined in question2 
    line_list = env.obs_line_states

    pf_force =  pf.perpendicular(line_list[0], env.robot.state, 0, 1, env.robot.state[1]>=6)
    pf_force += pf.perpendicular(line_list[1], env.robot.state, 0, 2, env.robot.state[1]>=6)
    pf_force += pf.perpendicular(line_list[2], env.robot.state, 220, -1, env.robot.state[0]>=6.5)
    pf_force += pf.attractive(env.robot.goal[0:2], env.robot.state, 2, env.robot.state[1]<=6 )
        
    env.robot_step(pf_force)
    env.render(show_traj=True)

    if env.collision_check() or env.arrive_check():  # check whether there are 
        break

if animation:
    env.save_ani(image_path, gif_path, ani_name='potential_field', keep_len=10)

print('done')
env.show()