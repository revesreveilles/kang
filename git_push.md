## 提交修改到上游分支
```
git stash
git pull --rebase
git stash pop
git add <update_file>
git commit -m "declare"
git push
```
提示当前分支没有跟踪信息，指定要变基到某一分支：
```
git branch --set-upstream-to=<remote_name>/<branch_name>
# 指定远程分支
git pull --rebase <remote_name> <branch_name>
```