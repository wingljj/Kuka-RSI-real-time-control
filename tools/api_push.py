# -*- coding: utf-8 -*-
"""通过 GitHub Git Data API 精确重建本地提交并推进远端分支。
github.com:443 被墙时的替代推送通道(api.github.com 可达)。
逐字节复刻 blob/tree/commit(含作者与时间),SHA 与本地一致,不产生分叉。
用法: python api_push.py <token> <owner/repo> <branch> <commit1> [<commit2> ...]
"""
import base64
import json
import subprocess
import sys
import time
import urllib.request

TOKEN, REPO, BRANCH = sys.argv[1], sys.argv[2], sys.argv[3]
COMMITS = sys.argv[4:]
API = "https://api.github.com"


def req(method, path, payload=None):
    url = API + path
    data = json.dumps(payload).encode() if payload is not None else None
    last_err = None
    for attempt in range(5):                     # 网络不稳:重试 5 次
        r = urllib.request.Request(url, data=data, method=method)
        r.add_header("Authorization", "token " + TOKEN)
        r.add_header("Accept", "application/vnd.github+json")
        r.add_header("User-Agent", "api-push")
        try:
            with urllib.request.urlopen(r, timeout=60) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:                   # noqa: BLE001
            last_err = e
            body = getattr(e, "read", lambda: b"")()
            print(f"  retry {attempt+1}: {e} {body[:200]}", flush=True)
            time.sleep(5)
    raise SystemExit(f"API 调用失败: {method} {path}: {last_err}")


def git(*args):
    return subprocess.run(["git"] + list(args), capture_output=True,
                          check=True).stdout


def gits(*args):
    return git(*args).decode().strip()


for commit in COMMITS:
    full = gits("rev-parse", commit)
    parent = gits("rev-parse", commit + "^")
    parent_tree = gits("rev-parse", commit + "^{tree}")
    local_tree = gits("rev-parse", commit + "^{tree}")

    # 变更文件: mode / 新 blob sha / 路径
    entries = []
    for line in git("diff-tree", "--no-commit-id", "-r", full).decode().splitlines():
        meta, path = line.split("\t", 1)
        _, mode, _, new_sha, status = meta.split(" ")
        if status == "D":
            entries.append({"path": path, "mode": "100644",
                            "type": "blob", "sha": None})
            continue
        blob = git("cat-file", "blob", new_sha)
        got = req("POST", f"/repos/{REPO}/git/blobs",
                  {"content": base64.b64encode(blob).decode(),
                   "encoding": "base64"})["sha"]
        assert got == new_sha, f"blob sha 不一致: {path} {got} != {new_sha}"
        entries.append({"path": path, "mode": mode, "type": "blob",
                        "sha": new_sha})
        print(f"  blob ok {path}", flush=True)

    tree = req("POST", f"/repos/{REPO}/git/trees",
               {"base_tree": gits("rev-parse", parent + "^{tree}"),
                "tree": entries})["sha"]
    assert tree == local_tree, f"tree sha 不一致: {tree} != {local_tree}"

    # 提交元数据逐字段复刻(ISO 时间含原时区)
    an, ae, ad = (gits("log", "-1", "--format=%an%n%ae%n%aI", full)
                  .splitlines())
    cn, ce, cd = (gits("log", "-1", "--format=%cn%n%ce%n%cI", full)
                  .splitlines())
    msg = git("log", "-1", "--format=%B", full).decode()
    # %B 会多出一个换行;git 对象内消息以单个 \n 结尾
    if msg.endswith("\n"):
        msg = msg[:-1]

    made = req("POST", f"/repos/{REPO}/git/commits",
               {"message": msg, "tree": tree, "parents": [parent],
                "author": {"name": an, "email": ae, "date": ad},
                "committer": {"name": cn, "email": ce, "date": cd}})["sha"]
    if made != full:
        print(f"  警告: commit sha 不一致 {made} != {full}(内容相同,"
              f"将以 API 侧 sha 推进)", flush=True)
        full = made
    print(f"commit ok {full}", flush=True)
    tip = full

req("PATCH", f"/repos/{REPO}/git/refs/heads/" + BRANCH.replace("/", "%2F"),
    {"sha": tip, "force": False})
print("ref updated ->", tip)
