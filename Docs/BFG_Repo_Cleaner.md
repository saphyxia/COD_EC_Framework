# BFG Repo-Cleaner

在项目开发的过程中，Repository的体积异常增大，其原因在于`.git`文件夹中保留了大量版本控制系统留下的历史提交记录和不必要的文件。为了减小Repository的体积、提高开发效率和整体性能，可以通过一些工具来进行“瘦身”。

BFG Repo-Cleaner是由开源社区构建和维护的工具。它提供了一种更快、更简单的`git filter-repo`替代方案，用于删除上述不需要的数据。

## 环境配置

* BFG Repo-Cleaner的运行依赖java，使用之前需要安装[java运行环境](https://www.java.com/en/)；

* 访问[BFG Repo-Cleaner](https://rtyley.github.io/bfg-repo-cleaner/)官网，下载`bfg-version.jar`

## 使用方法

### 建立源存储库的镜像

使用`Git Bash`工具，以[saphyxia/COD_EC_Framework](https://github.com/saphyxia/COD_EC_Framework)为例

```git
git clone --mirror https://github.com/saphyxia/COD_EC_Framework.git
```

### *统计磁盘消耗量

> 注：非必要步骤，仅为清理结束后再次执行以作对比展示

```git
git count-objects -vH
```

结果如下

```git
count: 8
size: 36.00 KiB
in-pack: 7145
packs: 1
size-pack: 162.60 MiB
prune-packable: 0
garbage: 0
size-garbage: 0 bytes
```

工程文件所占用的磁盘空间仅约为25M，而`.git`文件夹内的`.pack`文件占用160M余，对比印证出数据清理的必要性。

### 数据清理

将`bfg-version.jar`和`COD_EC_Framework`放置于同一目录下

```
...
  ├───...
  ├───COD_EC_Framework
  └───bfg-version.jar
```

考虑到当前工程文件内有效文件均为代码文件、脚本文件或数据结构文件，单个文件所占用的磁盘空间较小，此处设定清除在1M以上的文件(更多使用方法请参照[BFG Repo-Cleaner](https://rtyley.github.io/bfg-repo-cleaner/))，在`bfg-version.jar`同级目录执行

```git
java -jar bfg-version.jar --strip-blobs-bigger-than 1M COD_EC_Framework
cd COD_EC_Framework/
git reflog expire --expire=now --all && git gc --prune=now --aggressive
```

再次查看清理后占用的磁盘空间

```git
count: 0
size: 0 bytes
in-pack: 7167
packs: 1
size-pack: 30.67 MiB
prune-packable: 0
garbage: 0
size-garbage: 0 bytes
```

最后，推送到远端完成清理。