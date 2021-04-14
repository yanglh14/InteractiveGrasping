# Project Overview
This paper presents a novel design of a soft tactile finger with omni-directional adaptation using multi-channel optical fibers for rigid-soft interactive grasping. Machine learning methods are used to train a model for real-time prediction of force, torque, and contact using the tactile data collected. We further integrated such fingers in a reconfigurable gripper design with three fingers so that the finger arrangement can be actively adjusted in real-time based on the tactile data collected during grasping, achieving the process of rigid-soft interactive grasping. Detailed sensor calibration and experimental results are also included to further validate the proposed design for enhanced grasping robustness.

# Quick Start

## 1.clone this project 

```
$ git clone https://github.com/yanglh14/InteractiveGrasping.git
```

## 2. activate our virtual enviroment for some dependencies

### this part needs to install vitualenv first 

```
$ python -m pip3 install --user virtualenv
$ source my_env/bin/activate
```

## 3. run the function files up to your objective. 

**_ data_processing _** : process data collected and train the data with machine learning method. 

**_ visulization _** : visulize the data processing and experiment results.

**_ DataCollection _** : collect data for calibration.

**_ InteractiveGrasping _** : implement rigid-soft interactive grasping.

# Contact Information

Should you have some questions regarding the implement of our soft tactile sensor, please contact Yang Linhan at yanglh14@163.com .


