Git教程笔记
进入gitbase，右键Git Base Here
git init 		初始化仓库
git status 	查看状态
git add -A   	上传目录下的所有文件

提交流程
git add “上传文件名.txt”	     上传的文件
git commit -m “更新的信息”     当前文件的更新简介

git log 		查看日志
git diff 		查看改变
git checkout -- .  撤销更改
git reset –hard 63106fa    版本回退命令，后面数字为版本号前7位
git reflog  	查看版本修改记录
git clean -xf	删除未被跟踪的文件

git与Github关联使用
首先配置git Base
	git config --global user.name ""
	git config --global user.email ""

生成ssh key
	ssh-keygen -t rsa -C "邮箱地址"

测试是否连接成功
	ssh -T git@github.com

本地关联GitHub
	git remote add origin 项目的地址

重新抓取并合并远程仓库全部内容
	git pull origin master

上传当前更新 
	git push
