# GitHub Command Line
* Author: Zi-ang Cao
* Date: Apr 30, 2023
## Table of Content
- [GitHub Command Line](#github-command-line)
  - [Table of Content](#table-of-content)
    - [Setup Git SSH](#setup-git-ssh)
    - [Change/Create Git Branch](#changecreate-git-branch)
    - [Enforce Git Pull](#enforce-git-pull)
    - [Enforce Git Push](#enforce-git-push)
    - [Git Basic](#git-basic)


### Setup Git SSH
* [Guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux)
```Shell
# ssh-keygen -t rsa -b 4096 -C <GitHub Email>
ssh-keygen -t rsa -b 4096 -C "ZIC25@pitt.edu"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

git config --global user.email "ZIC25@pitt.edu"
git config --global user.name "Zi-ang Cao"
```

### Change/Create Git Branch
```Shell
# Create a new branch
git branch Ziang-debug

# Switch to this new branch
git checkout Ziang-debug

# Publish this new branch to online.
git push --set-upstream origin Ziang-debug
```


### Enforce Git Pull
```Shell
# Clone from a certain branch
git clone git@github.com:ice-bear-git/CMU_RI_16350_Course_Project.git -b ZiangCao


""" Overwrite the local change and enfore the pull from remote repository """
# Fetch the latest version of Code
git fetch --all
# Check current branch
git branch

# Pull from ZiangCao branch and Overwrite the local change
git reset --hard origin/master
git reset --hard origin/ZiangCao
```

### Enforce Git Push
```Shell
""" Use the -f or --force option with git push to enforce the change """
# Make sure to replace <remote_name> with the name of the remote repository you're pushing to, 
# and <branch_name> with the name of the branch you want to push to.
git push -f <remote_name> <branch_name>

# Here, "origin" = remote repository name; "ZiangCao" = branch name
git push -f origin ZiangCao
```

### Git Basic
```Shell
git status
git add .
git commit -m "commit message"
git push
```
