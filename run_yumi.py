#!/usr/bin/env python

from tasks.yumi_task import PickAndPlace


if __name__ == '__main__':


    pnp = PickAndPlace(
        place_position_l=[0.3, 0.4, 0.15, 0],
        place_position_r=[0.3, -0.4, 0.15, 0],
        hover_distance=0.07,
        step_size=0.01
    )
    pnp.run()
