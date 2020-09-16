# Use the Github to manage your project

## Abstract
The simplest services for hosting project files are Dropbox and Google Drive . It is very easy to get started with these systems, and they allow you to share files among laptops and mobile units with as many users as you want. The systems offer a kind of version control in that the files are stored frequently (several times per minute), and you can go back to previous versions for the last 30 days. However, it is challenging to find the right version from the past when there are so many of them and when the different versions are not annotated with sensible comments.

**[Git](https://github.com/weisongwen)** is designed to solve this kind of problem!

## Content
### Initialize your github respository and upload your code

- **global initialization**
    ```
    $ git config --global user.name "John Smith"
    $ git config --global user.email jsmith@seas.upenn.edu
    ```
<<<<<<< 5fe2b0a25cb14d933b4939b90a159a148bf47923
    if you want to use a different name/email for a specific project, you cna change it just for that package. use the above commands, but leave out the *--global*
=======
    if you want to use a different name/email for a specific project, you can change it just for that package. use the above commands, but leave out the *--global*
>>>>>>> add lab sensorkit extrinsics between stereo and imu
- **connect the respository in the internet**
    ```
    $ git remote add origin git@git.assembla.com:portfolio/space.space_name.git
    ```
- **submit your editing to the respository**
    - **step 1: add all**
    ```
    $ git add -A
    ```
    - **step 2: commit**
    ```
    $ git commit -m "what change you have done"
    ```
    - **step 3: push**
    ```
    $ git push -u origin master
    ```
- **Drag the latest files from the remote respository**
    - **step 1: pull all**
    ```
    $ git pull origin master
    ```
<<<<<<< 5fe2b0a25cb14d933b4939b90a159a148bf47923
=======

    - **[got conflict with the cloud end](https://blog.csdn.net/asty9000/article/details/83591142)** 
    ```bash
    #temporarily save the content causing the conflict
    git stash
    #pull the latest code from the cloud end
    git pull origin master
    #recover the content causing the conflict
    git stash pop stash@{0}
    # fix the conflict by hand checking

    # clear the content 
    git stash clear
    ```
>>>>>>> add lab sensorkit extrinsics between stereo and imu
- **generate new branch and push code to new branch**
    - **step 1: generate branch and push code**
    ```
    git branch test      (新建一个名称为"test"的分支)
    git checkout test      (从master切换到分支test下)
    git add .      
    git commit -m "描述"
    git push -u origin test      (将修改后的文件上传到github中)
    ```
    - **step 2: merge branch**
    ```  
    git checkout master     (从分支test下切换到主分支master下)
    git merge origin/xxx      (合并xxx分支到主分支，此处的xxx为test)
    git push -u origin master      (上传到github)
    ```
<<<<<<< 5fe2b0a25cb14d933b4939b90a159a148bf47923
=======
- **delete the lcoal and remote branchs**
    - **remote branch**
    ```
    git push origin --delete your_remote_branch
    ```
    - **local branch**
    ```
    git branch -d your_local_branch
    git branch -D your_local_branch
    ```
>>>>>>> add lab sensorkit extrinsics between stereo and imu
### Reference
1. [A tutorial for Git and GitLab](https://www.ifi.uzh.ch/dam/jcr:ff780599-d5e2-4d05-b923-1c333cbf2842/A%20Tutorial%20for%20GitHub.pdf)
2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)