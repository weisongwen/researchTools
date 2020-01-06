# Debug ros node with GDB 

## Abstract
Debug the ros node with GDB. Some error (such as **segment fault**) is difficult to find. The GDB is a good choice to debug it.

<!-- <p align="center">
  <img width="712pix" src="img/framework20200104.png">
</p> -->

## Simple steps to use GDB to debug ros node

- **Step 1**: compile the ros package
  ```
  $ catkin_make -DCMAKE_BUILD_TYPE=Debug
  ```
- **Step 2**: run the ros node you want to debug with specified format
  ros node
  ```
  $ rosrun --prefix 'gdb -ex run --args'  package   node  
  ```
  or (launch)
  ```
  $ <node pkg="package" type="node" name="node_name" output="screen" launch-prefix="xterm -e gdb -ex run --args">/>  
  ```
  or (python)
  ```
  launch-prefix="xterm -e python -m pdb "
  ```

## additional commands for GDB debugging

| commands | functions |
| :---:  | :---:  |
| bt | show location of error, e.g. **segment fault** |
| info files | show the detail information of the debugged file |
| info var | show all the global and static variables |
| info local | show all the local variables |

<!-- ```
├── self-supervised-depth-completion
├── data
|   ├── data_depth_annotated
|   |   ├── train
|   |   ├── val
|   ├── data_depth_velodyne
|   |   ├── train
|   |   ├── val
|   ├── depth_selection
|   |   ├── test_depth_completion_anonymous
|   |   ├── test_depth_prediction_anonymous
|   |   ├── val_selection_cropped
|   └── data_rgb
|   |   ├── train
|   |   ├── val
├── results
``` -->
