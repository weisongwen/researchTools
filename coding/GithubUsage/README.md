# Use the Github to manage your project

## Abstract
The simplest services for hosting project files are Dropbox and Google Drive . It is very easy to get started with these systems, and they allow you to share files among laptops and mobile units with as many users as you want. The systems offer a kind of version control in that the files are stored frequently (several times per minute), and you can go back to previous versions for the last 30 days. However, it is challenging to find the right version from the past when there are so many of them and when the different versions are not annotated with sensible comments.

**Git** is designed to solve this kind of problem!

## Content
### Initialize your github respository and upload your code

- **global initialization**
    ```
    $ git config --global user.name "John Smith"
    $ git config --global user.email jsmith@seas.upenn.edu
    ```
    if you want to use a different name/email for a specific project, you cna change it just for that package. use the above commands, but leave out the *--global*
- **connect the respository in the internet**
    ```
    $ git remote add origin git@git.assembla.com:portfolio/space.space_name.git
    ```
- **submit your editing to the respository**
    - **step 1: add all**
    ```
    $ git add -A
    ```
    - **step 2: commit
    ```
    $ git commit -m "what change you have done"
    ```
    - **step 3: push
    ```
    $ git push -u origin master
    ```
### Reference
1. [A tutorial for Git and GitLab](https://www.ifi.uzh.ch/dam/jcr:ff780599-d5e2-4d05-b923-1c333cbf2842/A%20Tutorial%20for%20GitHub.pdf)
2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)