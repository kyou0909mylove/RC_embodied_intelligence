# -*- coding: utf-8 -*-
"""串行、有限等待的记录写入。磁盘满/写入慢不抢断机器人动作。

主线程最多等待record_timeout_s；后台保留FIFO顺序。备用目录无法写入时
保留内存状态，明确警告，不承诺进程退出后可恢复。
"""
from pathlib import Path
from queue import Queue, Full
import copy
import hashlib
import os
import tempfile
import threading
from .checkpoint import atomic_json


def fallback_directory(root):
    tag = hashlib.sha256(str(Path(root).resolve()).encode()).hexdigest()[:12]
    return Path(tempfile.gettempdir())/('embodied-records-{}-{}'.format(os.getuid(),tag))


class RecordWriter:
    def __init__(self, fallback, warning):
        self.fallback,self.warning = Path(fallback),warning
        self.queue = Queue(maxsize=128)
        self.saved_paths = {}
        self.thread = threading.Thread(target=self._run,daemon=True,name='mission-record-writer')
        self.thread.start()

    @staticmethod
    def _save(kind,path,data):
        if kind=='json':
            atomic_json(path,data)
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('a',encoding='utf-8') as stream:
                stream.write(data)

    def _run(self):
        while True:
            kind,path,data,done,outcome = self.queue.get()
            try:
                self._save(kind,path,data)
                outcome['saved'] = True
                self.saved_paths[str(path)] = path
            except Exception as exc:
                self.warning('记录写入失败，尝试备用目录：'+str(exc))
                try:
                    common = path.name=='latest_match.json' or path.name.startswith(('checkpoint-','started-'))
                    alternate_path = self.fallback/path.name if common else self.fallback/path.parent.name/path.name
                    self._save(kind,alternate_path,data)
                    outcome['saved'] = True
                    self.saved_paths[str(path)] = alternate_path
                except Exception as alternate:
                    self.warning('备用记录也不可写，本进程仅保留内存状态：'+str(alternate))
            finally:
                done.set()
                self.queue.task_done()

    def _enqueue(self,kind,path,data,timeout):
        done,outcome = threading.Event(),{'saved':False}
        try:
            self.queue.put_nowait((kind,Path(path),copy.deepcopy(data),done,outcome))
        except Full:
            self.warning('记录队列已满，本次状态保留内存，未保证落盘')
            return False
        if timeout==0:
            return True  # 已排队不等于已落盘。
        if not done.wait(timeout):
            self.warning('记录写入超过{}秒，后台继续写入，主流程继续'.format(timeout))
            return False
        return outcome['saved']

    def write(self,path,data,timeout):
        return self._enqueue('json',path,data,timeout)

    def append(self,path,text):
        return self._enqueue('append',path,text,0)
