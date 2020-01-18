# Evaluating the performance of SLAM　(relative positioning) and sensor fusion (absolute positioning) using [evo](https://github.com/MichaelGrupp/evo)

***Python package for the evaluation of odometry and SLAM***

| Linux / macOS / Windows / ROS |
| :---: |
| [![Build Status](https://dev.azure.com/michl2222/michl2222/_apis/build/status/MichaelGrupp.evo?branchName=master)](https://dev.azure.com/michl2222/michl2222/_build/latest?definitionId=1&branchName=master) |

This package provides executables and a small library for handling, evaluating and comparing the trajectory output of odometry and SLAM algorithms.

Supported trajectory formats:

* 'TUM' trajectory files
* 'KITTI' pose files
* 'EuRoC MAV' (.csv groundtruth and TUM trajectory file)
* ROS bagfile with `geometry_msgs/PoseStamped`, `geometry_msgs/TransformStamped`, `geometry_msgs/PoseWithCovarianceStamped` or `nav_msgs/Odometry` topics

See [here](https://github.com/MichaelGrupp/evo/wiki/Formats) for more infos about the formats.

<a href="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/ape_demo_ORB_map.png" target="_blank">
  <img src="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/ape_demo_ORB_map.png" alt="evo" height="200" border="5" />
</a>
<a href="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/res_violin.png" target="_blank">
  <img src="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/res_violin.png" alt="evo" height="200" border="5" />
</a>
<a href="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/markers.png" target="_blank">
  <img src="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/markers.png" alt="evo" height="200" border="5" />
</a>
<a href="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/res_stats.png" target="_blank">
  <img src="https://raw.githubusercontent.com/MichaelGrupp/evo/master/doc/assets/res_stats.png" alt="evo" height="200" border="5" />
</a>

---

## installation

Installation is easy-peasy if you're familiar with this: https://xkcd.com/1987/#

**Python 3.5+** and **Python 2.7** are both supported. If you want to use the ROS bagfile interface, first check which Python version is used by your ROS installation and install accordingly.
You might also want to use a [virtual environment](https://github.com/MichaelGrupp/evo/blob/master/doc/install_in_virtualenv.md).

### From PyPi
If you just want to use the executables of the latest release version, the easiest way is to run:
```bash
pip install evo --upgrade --no-binary evo
```
This will download the package and its dependencies from [PyPI](https://pypi.org/project/evo/) and install or upgrade them. Depending on your OS, you might be able to use `pip2` or `pip3` to specify the Python version you want. Tab completion for Bash terminals is supported via the [argcomplete](https://github.com/kislyuk/argcomplete/) package on most UNIX systems - open a new shell after the installation to use it (without `--no-binary evo` the tab completion might not be installed properly). If you want, you can subscribe to new releases via https://libraries.io/pypi/evo.

### From Source
Run this in the repository's base folder:
```bash
pip install --editable . --upgrade --no-binary evo
```

### Bug list
- Cannot uninstall 'scipy'. It is a distutils installed project and thus we cannot accurately determine which files belong to it which would lead to only a partial uninstall.
    - solutions
        ```
        sudo pip install --upgrade scipy --ignore-installed six
        ```
### Useful commands

- **get error and visualize trajectory**
```
evo_rpe tum groundTruth.csv vio.csv -va --plot --plot_mode xyz // the relative position error between the reference trajactory with others

evo_traj tum groundTruth.csv vio.csv --plot  // plot the trajactory of the two .csv data

evo_rpe tum groundTruth.csv vio.csv -r angle_deg --delta 1 --delta_unit m -va --plot --plot_mode xyz --save_plot ./VINSplot --save_results ./VINS.zip

evo_traj tum groundTruth.csv vio.csv --plot --save_plot ./VINSplottraj1

evo_res result079 result063 result058 --use_filenames -p --save_plot ./multipleplot
```
- **Transfer the npz format to csv, typing code in terminal**
```
python
import numpy as np
data = np.load('/home/bxw/remoteSensing2020/src/result/error_array.npz') // change the path to your file
print(data) // the 4th steps, it is just print the data in terminal
np.savetxt("error.csv",data,delimiter=",") //npz to csv file

```

### The other using, for example;
```
evo_rpe tum groundTruth.csv vio.csv -va --t_offset 0.0 --plot --plot_mode xyz 

evo_config set plot_seaborn_style whitegrid 
这是修改背景，其中whitegrid可以替换成darkgrid, whitegrid, dark, white, ticks
evo_config set plot_seaborn_palette bright
这是修改线条，其中bright可以替换成deep, muted, bright, pastel, dark, colorblind

evo_rpe tum groundTruth.csv vio.csv -r angle_deg --plot (google the use of -r angle_deg )

evo_traj tum groundTruth.csv vio.csv --ref groundTruth.csv (google the use of --ref)
```

### **Temporal alignment**
This is done by searching the best matching timestamps between the reference and the other trajectories. All trajectories are then reduced to the best matching timestamps. If no matches are found, an error is thrown. The options for this step are:
```
--t_offset: add a constant timestamp offset (not adding to --ref trajectory) - default: 0.0s
--t_max_diff: maximum timestamp difference for data association - default: 0.01s
```
If you give a reference trajectory with --ref, you can align the other trajectories to the reference with Umeyama's method:
```
--align or -a = SE(3) Umeyama alignment (rotation, translation)
--align --correct_scale or -as = Sim(3) Umeyama alignment (rotation, translation, scale)
--correct_scale or -s = scale correction only
```

### Reference
1. [evo](https://github.com/MichaelGrupp/evo)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)