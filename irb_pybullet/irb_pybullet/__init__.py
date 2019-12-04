from gym.envs.registration import register

register(
    id='irb_pybullet-v0',
    entry_point='irb_pybullet.envs:OpenaiIRB')


