# DeepMimic Bullet Enviroment

### Inverse Kinect and Reference Rendering

If you want to learn movements directly from coordinates, you can use **inverse_kinect.py** and **reder_reference.py**, currently it just support a dataset created by [VideoPose3D](https://github.com/facebookresearch/VideoPose3D).

Download the pre-prosessed Human3.6M dataset of Videopose3D at [here](https://www.dropbox.com/s/z5bwig0h6mww590/data_3d_h36m.npz?dl=0).

If you created a **data/** directory in **deep_mimic/**, copeid the data_3d_h36m.npz file to this directory, then you can try to run this command: 

```shell
$ python render_reference.py \
	--dataset_path=data/data_3d_h36m.npz \
    --fps=24 \
    --subject=S11 \
    --action=Walking \
    --loop=wrap  \
    --json_path=data/Walking.json \
    --draw_gt
```

"--draw_gt" will draw the ground truth using **pybullet.addUserDebugLine()**, the right part of the humanoid lines will be red, other parts will be black. This is just for debugging, the render process will be much faster without the '--draw_gt' flag.

### Contact

Inverse kinect and reference rendering module is developed by Somedaywilldo.

Email: somedaywilldo@foxmail.com

### Reference

[1] DeepMimic: Example-Guided Deep Reinforcement Learning of Physics-Based Character Skills [[Link](https://arxiv.org/abs/1804.02717)]

[2] 3D human pose estimation in video with temporal convolutions and semi-supervised training [[Link](https://arxiv.org/abs/1811.11742)]



