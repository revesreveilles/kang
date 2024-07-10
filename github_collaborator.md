# 实现Github仓库远程合作
在要实现的远程仓库的页面下点击`Settings`-->`Collaborators`
在`Manage access`下点击`Add people`
```
cd <工作空间>
git init
ssh-keygen -t rsa -C "your@email.com"
```
## 添加ssh认证
1. <a herf=https://github.com//>点击进入Github</a>.
2. 点击头像-->`SSH adn GPG keys`-->'New SSH key'-->
在终端输入`ssh-keygen -t rsa -C "your@email.com"`生成的密钥默认存放于`/home/<username>/.ssh`,复制`id_rsa.pub`内的全部内容上去，点击`Generate new ssh key`
```
git checkout -b <分支名>
git remote add <remote名> <要加入的远程仓库的SSH>
git add <提交文件名>
git commit -m "提交描述"
git push --set-upstream <remote名> <分支名>
```
