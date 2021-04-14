![DeepClaw-Logo](docs_md/docs/asset/fig-DeepClaw.png)

# The DeepClaw Benchmark

The DeepClaw is a benchmarking model zoo that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. The main homepage can be found at [here](https://bionicdl-sustech.github.io/DeepClaw/). This is the GitHub repository of DeepClaw source code, including instructions for installing and using DeepClaw.

## Resources

- Documentation: https://bionicdl-sustech.github.io/DeepClaw/
- Paper explaining DeepClaw: [arXiv:2005.02588 [cs.RO]](https://arxiv.org/abs/2005.02588)
- Papers using DeepClaw: 
    - [arXiv:2003.01584 [cs.RO]](https://arxiv.org/abs/2003.01584)
    - [arXiv:2003.01583 [cs.RO]](https://arxiv.org/abs/2003.01583)
    - [arXiv:2003.01582 [cs.RO]](https://arxiv.org/abs/2003.01582)

## Code Organization

The DeepClaw code is organized as follows:

    configs/                        Configuration for robotic station for manipulation tasks.
    data/                           Common dataset.
    deepclaw/drivers/               Drivers for various robotic hardware, i.e. ur, franka, aubo.
    deepclaw/modules/               Model zoo for segmentation, classification, pick planning, and motion planning.
    deepclaw/sim2real/simulation    Robot simulations using Coppelism and PyRep.   
    deepclaw/utils/                 Server setup with dockers and client setup for laptops (x86) and jetson (arm).
    projects/                       Research projects using deepclaw.
    docs/                           Description of deepclaw as a manual.
    

## Bibliography

```
@misc{wan2020deepclaw,
    title={DeepClaw: A Robotic Hardware Benchmarking Platform for Learning Object Manipulation},
    author={Fang Wan and Haokun Wang and Xiaobo Liu and Linhan Yang and Chaoyang Song},
    year={2020},
    eprint={2005.02588},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```
