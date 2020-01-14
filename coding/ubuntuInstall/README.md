# Installing ubuntu in one disk (the disk already have an Windows operation system)

## Bugs and solutions
- **partitions in windows**
    some of the system, the disk is encrypted. Fristly, do the un-encryption
- **black scree when installing ubuntu**
push "e" and get into the vim to set the following
    ```
    $ ...splash nouveau.modeset=0 ---
    ```
- **partitions in ubuntu**
    - **space "/"**
    ```
    $ majority of the space. In fact, the "/" and the "home" share the same memory.
    ```
    - **bootload directory for ubuntu**
    ```
    $ use the default directory
    ```
    - **step 3: install graphical card**
    ```
    1  sudo apt-get update
    2  sudo apt-get upgrade
    3  #for case1: original driver installed by apt-get:
    4  sudo apt-get remove --purge nvidia*
    5  #for case2: original driver installed by runfile:
    6  sudo update-initramfs -u
    7  sudo add-apt-repository ppa:graphics-drivers/ppa
    8  sudo apt-get update
    9  sudo apt-get install nvidia-418
   10  sudo apt-get install mesa-common-dev
   11  sudo cp /etc/apt/sources.list /etc/apt/sources_init.list
   12  sudo mv /etc/grub.d/30_os-prober /etc/grub.d/08_os-prober
   13  sudo apt-get update
   14  sudo apt-get upgrade
   15  sudo update-grub
   16  sudo apt-get update
    ```

### Reference
1. [RMB 50, help to install the ubuntu remotely](https://detail.tmall.com/item.htm?id=599552476862&spm=a1z09.2.0.0.41c82e8dZDKFCA&_u=e2eoug0q53ca)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)