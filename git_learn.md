# git 学习 笔记　(2020 01 26)：

<center>
<img src="gitpic&book/Screenshot from 2021-02-01 10-26-21.png" alt="Terminator" width="８0%">
</center>
---
## １：  安装　sudo apt-get install git

```bash
# 版本查看
$ git --version
#  设置相关信息　,必须设置
$ git config --global user.name "Your Name"
$ git config --global user.email "email@example.com
```
---


## 2:   创建仓库 repository.

理解成一个目录，这个目录里面的所有文件都可以被Git管理起来，每个文件的修改、删除，Git都能跟踪，以便任何时刻都可以追踪历史，或者在将来某个时刻可以“还原”。

```bash
# 先进入需要创建仓库的文件夹
$ git init
Initialized empty Git repository in /Users/michael/learngit/.git/
```
---

## ３：  把文件添加到版本库 / git 结构介绍

```bash
# add 把文件添加到仓库，
$ git add readme.txt
# 将当前文件夹的所有文件添加到git仓库中
$ git add . 
#　commit 把文件提交到仓库
$ git commit -m "wrote a readme file"
[master (root-commit) eaadf4e] wrote a readme file
1 file changed, 2 insertions(+)
create mode 100644 readme.txt
```  
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 10-05-57.png" alt="Terminator" width="８0%">
<div>图：　本地git的关系</div>
</center>

### 3.1：  工作区（working diretory）－－＞和暂存区（Stage）－－＞和当前分支beanch

**工作区**：就是你在电脑里能看到的目录，还包括当前修改但未add存入暂存区的文件变化信息

**暂存区／缓冲区**：临时存储文件的变化信息,英文叫 stage 或 index。一般存放在 .git 目录下的 index 文件（.git/index）中，

**版本库**：工作区有一个隐藏目录 .git，这个不算工作区，而是 Git 的版本库。

第一步是用git add把文件添加进去，实际上就是把文件(或者修改)添加到暂存区；，暂存区中将记录file文件上的修改信息。

第二步是用git commit提交更改，实际上就是把暂存区的所有内容提交到当前分支。

!!commit：往往是有重大改变的版本或者是在一次修改工作整体完成之后才使用commit。而在这之间需要保存的修改，自然需要一个缓存区暂时存放。

因为我们创建Git版本库时，Git自动为我们创建了唯一一个master分支，所以，现在，git commit就是往master分支上提交更改。你可以简单理解为，需要提交的文件修改通通放到暂存区，然后，一次性提交暂存区的所有修改。

---

### 3.2 三棵树
https://git-scm.com/book/zh/v2/Git-%E5%B7%A5%E5%85%B7-%E9%87%8D%E7%BD%AE%E6%8F%AD%E5%AF%86
看这个就够了！！！！

|  树   | 用途  |
|  ----  | ----  |
| HEAD  | 上一次提交的快照，下一次提交的父结点 |
| Index  | 预期的下一次提交的快照 |
|Working Directory | 沙盒｜

**HEAD**
HEAD 是当前分支引用的指针，它总是指向该分支上的最后一次提交。 这表示 HEAD 将是下一次提交的父结点。 通常，理解 HEAD 的最简方式，就是将它看做 该分支上的最后一次提交 的快照.
```bash
# 用gitk查看一样的
$ git cat-file -p HEAD
tree 3133bad03c56b234b2ee562c8cafa6feb85f3ef1
parent d9bd2bbfd8f37b55b65d57da78df56856b577726
author yyxiong <429799862@example.com> 1611884508 +0800
committer yyxiong <429799862@example.com> 1611884508 +0800
```
**索引 index**

索引是你的 预期的下一次提交。 我们也会将这个概念引用为 Git 的“暂存区”，这就是当你运行 git commit 时 Git 看起来的样子。

**工作目录**

自己的 工作目录（通常也叫 工作区）。 另外两棵树以一种高效但并不直观的方式，将它们的内容存储在 .git 文件夹中。 工作目录会将它们解包为实际的文件以便编辑。 你可以把工作目录当做 沙盒。在你将修改提交到暂存区并记录到历史之前，可以随意更改。

---


## 4:    git status 
掌握仓库当前的状态,有哪些是被修改的（add以后），有哪些没有被提交（comment）
```bash        
$ git status
# readme.txt被修改了,也add了，但是没有被提交commit，被保存在缓存区
On branch master
Changes to be committed:
(use "git reset HEAD <file>..." to unstage)
         modified:   readme.txt

# 意思是，readme被提交到仓库，已经是一个新版本了．
$ git commit -m "add distributed"
[master e475afc] add distributed
1 file changed, 1 insertion(+), 1 deletion(-)

# 没有东西在缓存区．
$ git status
On branch master
nothing to commit, working tree clean
```

---

## ５：查看版本，版本退回 git log git reset and git checkout

**让readme.txt变成不同时候修改的内容，HEAD 是一个指针**
```bash
# 查看被提交的版本日志
$ git log　　
# commit 版本号
commit 1094adb7b9b3807259d8cb349e7df1d4d6477073 (HEAD -> master)　
Author: Michael Liao <askxuefeng@gmail.com>
 Date:   Fri May 18 21:06:15 2018 +0800

        append GPL

commit e475afc93c209a690c39c13a46716e8fa000c366
Author: Michael Liao <askxuefeng@gmail.com>
Date:   Fri May 18 21:03:36 2018 +0800

        add distributed

commit eaadf4e385e865d25c48e7ca9c8395c3f7dfaef0
Author: Michael Liao <askxuefeng@gmail.com>
Date:   Fri May 18 20:59:18 2018 +0800

        wrote a readme file

```

**第 1 步：移动 HEAD**

reset 做的第一件事是移动 HEAD 的指向。 这与改变 HEAD 自身不同（checkout 所做的）；reset 移动 HEAD 指向的分支。 这意味着如果 HEAD 设置为 master 分支（例如，你正在 master 分支上）， 运行 git reset 9e5e6a4 将会使 master 指向 9e5e6a4。

<center>
<img src="gitpic&book/Screenshot from 2021-01-29 11-01-35.png" alt="Terminator" width="50%">
<div>图：　本地git的关系</div>
</center>

使用 reset --soft，它将仅仅停在那儿。

现在看一眼上图，理解一下发生的事情：它本质上是撤销了上一次 git commit 命令。 当你在运行 git commit 时，Git 会创建一个新的提交，并移动 HEAD 所指向的分支来使其指向该提交。 当你将它 reset 回 HEAD~（HEAD 的父结点）时，其实就是把该分支移动回原来的位置，而不会改变索引和工作目录。

**第 2 步：更新索引（--mixed）**

注意，如果你现在运行 git status 的话，就会看到新的 HEAD 和以绿色标出的它和索引之间的区别。

接下来，reset 会用 HEAD 指向的当前快照的内容来更新索引。

<center>
<img src="gitpic&book/Screenshot from 2021-01-29 11-05-45.png" alt="Terminator" width="50%">
<div>图：　本地git的关系</div>
</center>

如果指定 --mixed 选项，reset 将会在这时停止。 **这也是默认行为**，所以如果没有指定任何选项（在本例中只是 **git reset HEAD~**），这就是命令将会停止的地方。

现在再看一眼上图，理解一下发生的事情：它依然会撤销一上次 提交，但还会 取消暂存 所有的东西。 于是，我们回滚到了所有 git add 和 git commit 的命令执行之前。

**第 3 步：更新工作目录（--hard）**

reset 要做的的第三件事情就是让工作目录看起来像索引。 **如果使用 --hard 选项**，它将会**继续**这一步。
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 11-07-38.png" alt="Terminator" width="50%">
<div>图：　本地git的关系</div>
</center>

现在让我们回想一下刚才发生的事情。 你撤销了最后的提交、git add 和 git commit 命令 以及 工作目录中的所有工作。

必须注意，

**--hard 标记是 reset 命令唯一的危险用法**，

它也是 Git 会真正地销毁数据的仅有的几个操作之一。 其他任何形式的 reset 调用都可以轻松撤消，但是 --hard 选项不能，因为它强制覆盖了工作目录中的文件。 在这种特殊情况下，我们的 Git 数据库中的一个提交内还留有该文件的 v3 版本， 我们可以通过 reflog 来找回它。但是若该文件还未提交，Git 仍会覆盖它从而导致无法恢复。


**回顾**

reset 命令会以特定的顺序重写这三棵树，在你指定以下选项时停止：

    1:移动 HEAD 分支的指向 （若指定了 --soft，则到此停止）

    2:使索引index看起来像 HEAD （若指定 --mixed 或者　不指定，则到此停止）

    3:使工作目录看起来像索引　（若指定了 --hard，则到此停止）

举例子：
```bash
#　上一个版本 HEAD^ HEAD~1  
#  上上一个版本  HEAD^^ HEAD^2  
$ git reset --hard HEAD^
HEAD is now at e475afc add distributed
                
$ git log　　# 查看被提交的版本日志                
commit e475afc93c209a690c39c13a46716e8fa000c366
Author: Michael Liao <askxuefeng@gmail.com>
Date:   Fri May 18 21:03:36 2018 +0800

        add distributed

commit eaadf4e385e865d25c48e7ca9c8395c3f7dfaef0
Author: Michael Liao <askxuefeng@gmail.com>
Date:   Fri May 18 20:59:18 2018 +0800

        wrote a readme file
                   
# 再变成　append GPL　版本
$ git reset --hard 1094a　　//1049a是　commit id号
HEAD is now at 83b0afe append GPL
      
# 如果不知道版本号，用reflog查看，相当于bash中的history
$ git reflog
e475afc HEAD@{1}: reset: moving to HEAD^
1094adb (HEAD -> master) HEAD@{2}: commit: append GPL
e475afc HEAD@{3}: commit: add distributed
eaadf4e HEAD@{4}: commit (initial): wrote a readme file
```
**通过路径来重置**

我们运行 git reset file.txt

 （这其实是 git reset --mixed HEAD file.txt 的简写形式，

 将 file.txt 从 HEAD 复制到索引中。

 它还有 取消暂存文件 的实际效果。 如果我们查看该命令的示意图，然后再想想 git add 所做的事，就会发现它们正好相反。

<center>
<img src="gitpic&book/Screenshot from 2021-01-29 11-22-32.png" alt="Terminator" width="50%">
<div>图：　本地git的关系</div>
</center>

<center>
<img src="gitpic&book/Screenshot from 2021-01-29 11-23-11.png" alt="Terminator" width="50%">
<div>图：　本地git的关系</div>
</center>

这就是为什么 git status 命令的输出会建议运行此命令来取消暂存一个文件。 

---

## 6:  修改管理

第一次修改 -> git add -> 第二次修改 -> git commit

Index没有变化，第二次的修改不会被提交。
            

第一次修改 -> git add -> 第二次修改 -> git add -> git commit  

就相当于把两次修改合并后一块提交了,每次修改，如果不用git add到暂存区，那就不会加入到commit中。

---

## 7:  撤销修改

１：没有git add过，只是增加了一些然后保存．Git会告诉你，git checkout -- file可以丢弃工作区的修改 (将index信息复制到工作区)

```bash
$ git checkout -- readme.txt　　
# 1 readme.txt自修改后还没有被放到暂存区，现在，撤销修改就回到和版本库一模一样的状态；
# 2 readme.txt已经添加到暂存区后，又作了修改，现在，撤销修改就回到添加到暂存区后的状态。
# git checkout -- file命令中的--很重要，没有--，就变成了“切换到另一个分支”的命令，我们在后面的分支管理中会再次遇到git checkout命令。
```

２：你不但写了一些胡话，还git add到暂存区了，用命令git reset HEAD <file>可以把暂存区的修改撤销掉（unstage）：  
```bash
$ git reset HEAD readme.txt
Unstaged changes after reset:
M	readme.txt   
```    
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 11-22-32.png" alt="Terminator" width="50%">
<div>图：　本地git的关系</div>
</center>  

git reset命令既可以回退版本，也可以把暂存区的修改回退到工作区。当我们用HEAD时，表示最新的版本。然后再　git checkout -- readme.txt    

---


## 8: git remote远程仓库的添加、查看、修改、删除  -- github

```        
### 关于建立远程仓库，github 的提示
…or create a new repository on the command line
```bash
echo "# test" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/yyxiongtudresden/test.git
git push -u origin main
```
…or push an existing repository from the command line
```bash
git remote add origin https://github.com/yyxiongtudresden/test.git
git branch -M main
git push -u origin main
```


### 1: 添加远程仓库 & 查看远程仓库

前提是你已经将本地的git配置好了，git commit 完成以后，能够向github上传，在github上建立一个仓库，找到她的url

git remote
运行 git remote add <shortname\> <url\> 添加一个新的远程 Git 仓库<url\> ，同时指定一个方便使用的简写 <shortname\> ：


```bash
# git remote add “远程仓库名”+空格+远端仓库链接   
$ git remote add origin https://github.com/yyxiongtudresden/sensorfusion_deepblue_commentversion.git  
# 远程仓库的名称一般默认为 origin（远程仓库的名称推荐使用默认的名称 origin ）,
```

```bash
#git remote　列出每个远程库的简短名字，在克隆完某个项目后，至少可以看到一个名为 origin 的远程库,，git 默认使用这个名字来标识你所克隆的原始仓库。

# -v 会显示需要读写远程仓库使用的 Git 保存的简写与其对应的 URL。
$ git remote -v
origin	https://github.com/yyxiongtudresden/sensorfusion_deepblue_commentversion.git (fetch)
origin	https://github.com/yyxiongtudresden/sensorfusion_deepblue_commentversion.git (push)

# git 查看远程仓库，以及与本地仓库的关系
$ git remote show origin　
* remote origin
  Fetch URL: https://github.com/yyxiongtudresden/sensorfusion_deepblue_commentversion.git
  Push  URL: https://github.com/yyxiongtudresden/sensorfusion_deepblue_commentversion.git
  HEAD branch: main
  Remote branch:
    main tracked
  Local branch configured for 'git pull':
    main merges with remote main
  Local ref configured for 'git push':
    main pushes to main (up to date)
```

### 2:修改远程仓库的关联
```bash
# git remote set-url origin ＂新的仓库链接＂ 命令，更新远程仓库的 url。
$ git remote set-url origin "https://github.com/yyxiongtudresden/TEST.git"
```
### 3: fetch ^&
### 4:删除远程仓库  (感觉和３差不多)
```bash
# git remote remove　＂要删除的远程仓库名＂
git remote remove test
```

### 补充: 删除github上的文件夹

进入文件夹，settings -- 拖到底　－－danger zone.

---
## 9:  分支管理
参考资料：
https://git-scm.com/book/zh/v2/Git-%E5%88%86%E6%94%AF-%E5%88%86%E6%94%AF%E7%9A%84%E6%96%B0%E5%BB%BA%E4%B8%8E%E5%90%88%E5%B9%B6

### 1:查看所有本地分支
```bash
# 当前分支前面会标一个*号
git branch  
* main
```
### ２：查看本地和远程仓库的所有分支
```bash
git branch -a
```
输出如下：　（为了加颜色我放在了bash脚本之外）

*<font color="#00dd00">main</font><br /> 
 <font color="#dd0000">remotes/origin/main</font><br /> 


### ３：创建一个分支（dev)，并转到这个分支
前提：
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-23-59.png" alt="Terminator" width="50%">
<div>一个简单提交历史</div>
</center>

现在，你已经决定要解决系统中的 #53 问题。 新建一个分支并同时切换到那个分支上，你可以运行一个带有 -b 参数的 git checkout 命令：
```bash
$ git checkout -b iss53
Switched to a new branch "iss53"
```
它是下面两条命令的简写：
```bash
$ git branch iss53
$ git checkout iss53
```
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-30-16.png" alt="Terminator" width="60%">
<div>创建一个新分支指针</div>
</center>

你继续在 #53 问题上工作，并且做了一些提交。 在此过程中，iss53 分支在不断的向前推进，因为你已经检出到该分支 （也就是说，你的 HEAD 指针指向了 iss53 分支）
```bash
$ vim index.html
$ git commit -a -m 'added a new footer [issue 53]'
```
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-31-46.png" alt="Terminator" width="50%">
<div>iss53 分支随着工作的进展向前推进</div>
</center>

```bash
#  #Git的switch命令是2.23版本发布的新命令
git switch -c dev 
# 它等于下面两个
git branch dev && git switch dev　　
```
 <font color="#dd0000">但是，在你这么做之前，要留意你的工作目录和暂存区里那些还没有被提交的修改， 它可能会和你即将检出的分支产生冲突从而阻止 Git 切换到该分支。 最好的方法是，在你切换分支之前，保持好一个干净的状态。</font><br />    


### 4: 切换到已有分支　
```bash
git checkout xxx #在这个分支修改的东西和其它分支无关
git switch xxx　　#Git的switch命令是2.23版本发布的新命令
```
因为创建、合并和删除分支非常快，所以Git鼓励你使用分支完成某个任务，合并后再删掉分支，这和直接在master分支上工作效果是一样的，但过程更安全。

现在你接到那个电话，有个紧急问题等待你来解决。 有了 Git 的帮助，你不必把这个紧急问题和 iss53 的修改混在一起， 你也不需要花大力气来还原关于 53# 问题的修改，然后再添加关于这个紧急问题的修改，最后将这个修改提交到线上分支。 你所要做的仅仅是切换回 master 分支。
```bash
$ git checkout master
Switched to branch 'master'
```
这个时候，你的工作目录和你在开始 #53 问题**之前**一模一样，现在你可以专心修复紧急问题了。 你要修复这个紧急问题。 我们来建立一个 hotfix 分支，在该分支上工作直到问题解决：
```bash
$ git checkout -b hotfix
Switched to a new branch 'hotfix'
$ vim index.html
$ git commit -a -m 'fixed the broken email address'
[hotfix 1fb7853] fixed the broken email address
 1 file changed, 2 insertions(+)
 ```
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-37-41.png" alt="Terminator" width="50%">
<div>基于 master 分支的紧急问题分支 hotfix branch</div>
</center>


### ５: 分支的快进　
```bash
git merge 当前分支名，
#这样就把当前分支的东西修改到主分支中了． 
```
你可以运行你的测试，确保你的修改是正确的，然后将 hotfix 分支合并回你的 master 分支来部署到线上。 你可以使用 git merge 命令来达到上述目的：

```bash
$ git checkout master
$ git merge hotfix
Updating f42c576..3a0874c
Fast-forward
 index.html | 2 ++
 1 file changed, 2 insertions(+)
 ```
你应该注意到了“快进（**fast-forward**）” 这个词。 由于你想要合并的分支 hotfix 所指向的提交 C4 是你所在的提交 C2 的直接后继， 因此 Git 会直接将指针向前移动。换句话说，当你试图合并两个分支时， 如果顺着一个分支走下去能够到达另一个分支，那么 Git 在合并两者的时候， **只会简单的将指针向前推进（指针右移）**，因为这种情况下的合并操作没有需要解决的分歧——这就叫做 “快进（fast-forward）”。
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-40-34.png" alt="Terminator" width="50%">
<div>master 被快进到 hotfix</div>
</center>

### ６: 删除分支　git branch -d xxx

然而，你应该先删除 hotfix 分支，因为你已经不再需要它了 —— master 分支已经指向了同一个位置。 你可以使用带 -d 选项的 git branch 命令来删除分支：
```bash
$ git branch -d hotfix
Deleted branch hotfix (3a0874c).
```
经常会有冲突，不能将branch上的东西merge到main中．r
如果在main和某个branch 中同时修改了东西，那么就会产生冲突．
Git merge reports “Already up-to-date” though there is a difference
https://stackoverflow.com/questions/634546/git-merge-reports-already-up-to-date-though-there-is-a-difference

现在你可以切换回你正在工作的分支继续你的工作，也就是针对 #53 问题的那个分支（iss53 分支）。

```bash
$ git checkout iss53
Switched to branch "iss53"
$ vim index.html
$ git commit -a -m 'finished the new footer [issue 53]'
[iss53 ad82d7a] finished the new footer [issue 53]
1 file changed, 1 insertion(+)
```
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-42-49.png" alt="Terminator" width="50%">
<div>继续在 iss53 分支上的工作</div>
</center>

```bash
git checkout master
git reset --hard test  //强行把某个branch 设置为main branch 
```

### ７: 分支查看／管理
```bash
$ gitk 
```
### 8: 分支的合并　

假设你已经修正了 #53 问题，并且打算将你的工作合并入 master 分支。 为此，你需要合并 iss53 分支到 master 分支，这和之前你合并 hotfix 分支所做的工作差不多。 你只需要检出到你想合并入的分支，然后运行 git merge 命令

```bash
$ git checkout master
Switched to branch 'master'
$ git merge iss53
Merge made by the 'recursive' strategy.
index.html |    1 +
1 file changed, 1 insertion(+)
```
这和你之前合并 hotfix 分支的时候看起来有一点不一样。 在这种情况下，你的开发历史从一个更早的地方开始分叉开来（diverged）。 因为，master 分支所在提交并不是 iss53 分支所在提交的直接祖先，Git 不得不做一些额外的工作。 出现这种情况的时候，Git 会使用两个分支的末端所指的快照（C4 和 C5）以及这两个分支的公共祖先（C2），做一个简单的三方合并
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 13-53-22.png" alt="Terminator" width="50%">
<div> 一次典型合并中所用到的三个快照(这是合并之前)</div>
</center>

和之前将分支指针向前推进所不同的是，Git 将此次三方合并的结果做了一个新的快照并且自动创建一个新的提交指向它。 这个被称作一次合并提交，它的特别之处在于他有不止一个父提交。
<center>
<img src="gitpic&book/Screenshot from 2021-01-29 14-00-15.png" alt="Terminator" width="50%">
<div> 一个合并提交</div>
</center>

### ８：冲突管理
有时候合并操作不会如此顺利。 如果你在两个不同的分支中，对**同一个文件的同一个部分**进行了不同的修改，Git 就没法干净的合并它们。 如果你对 #53 问题的修改和有关 hotfix 分支的修改都涉及到同一个文件的同一处，在合并它们的时候就会产生合并冲突：
```bash
$ git merge iss53
Auto-merging index.html
CONFLICT (content): Merge conflict in index.html
Automatic merge failed; fix conflicts and then commit the result.
```
 你可以在合并冲突后的任意时刻使用 git status 命令来查看那些因包含合并冲突而处于未合并（unmerged）状态的文件：

```bash
$ git status
On branch master
You have unmerged paths.
  (fix conflicts and run "git commit")

Unmerged paths:
  (use "git add <file>..." to mark resolution)

    both modified:      index.html

no changes added to commit (use "git add" and/or "git commit -a")
```

Git 会在有冲突的文件中加入标准的冲突解决标记，这样你可以打开这些包含冲突的文件然后手动解决冲突。 出现冲突的文件会包含一些特殊区段，看起来像下面这个样子：
```bash
<<<<<< HEAD:index.html
<div id="footer">contact : email.support@github.com</div>
=======
<div id="footer">
 please contact us at support@github.com
</div>
>>>>>>> iss53:index.html
```
 为了解决冲突，你必须选择使用由 ======= 分割的两部分中的一个，或者你也可以自行合并这些内容。 例如，你可以通过把这段内容换成下面的样子来解决冲突：

```bash
<div id="footer">
please contact us at email.support@github.com
</div>
```
上述的冲突解决方案仅保留了其中一个分支的修改，并且 <<<<<<< , ======= , 和 >>>>>>> 这些行被完全删除了。 在你解决了所有文件里的冲突之后，<font color="#dd0000">对每个文件使用 git add 命令来将其标记为冲突已解决</font><br />。 一旦暂存这些原本有冲突的文件，Git 就会将它们标记为冲突已解决。
   

两个分支都提交了新的东西，用　git merge 会出现CONFLICT (content): Merge conflict in readme.txt
Git用<<<<<<<，=======，>>>>>>>标记出不同分支的内容，我们修改成一样的后保存．
    
!!!! 最好不要在两个brnach中都提交新的东西．不然就会造成冲突．
     
---    
##  10: 标签管理
        tag就是一个让人容易记住的有意义的名字，它跟某个commit绑在一起。
        １：在某个branch上打标签
            git tag <tagname>用于新建一个标签，默认为HEAD，也可以指定一个commit id；
            git tag v1.0
            如果要推送某个标签到远程，使用命令git push origin <tagname>　：　git push origin v1.0
        ３：查看标签信息
            git show v0.9
        ４：删除本地标签
            git tag -d v0.1
        ５：
