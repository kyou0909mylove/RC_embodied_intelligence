# -*- coding: utf-8 -*-
"""参数报错输出源文件、JSON路径和行号；软件恢复保留完整异常链。"""
import json
import re
import traceback


def _key_locations(text):
    """按JSON结构扫描键位置；重复detect_pose也能区分点号和柜层下标。"""
    decoder, locations = json.JSONDecoder(),[]
    def space(index):
        while index<len(text) and text[index].isspace():
            index += 1
        return index
    def walk(index,path):
        index = space(index)
        if text[index]=='{':
            index = space(index+1)
            while text[index]!='}':
                start = index
                key,end = decoder.raw_decode(text,index)
                child = path+'.'+key if path else key
                locations.append((child,text.count('\n',0,start)+1))
                index = walk(space(end)+1,child)
                index = space(index)
                if text[index]==',':
                    index = space(index+1)
                else:
                    break
            return index+1
        if text[index]=='[':
            index,item = space(index+1),0
            while text[index]!=']':
                index = space(walk(index,path+'['+str(item)+']'))
                item += 1
                if text[index]==',':
                    index = space(index+1)
                else:
                    break
            return index+1
        return decoder.raw_decode(text,index)[1]
    walk(0,'')
    return locations


def configuration_error(exc, path, root):
    """local覆盖模板。缺失键只报应填写路径，不编造行号。"""
    path,template = path.resolve(),(root/'config/navigation.example.json').resolve()
    sources, configs = {},{}
    for source in dict.fromkeys((path,template)):
        try:
            text = source.read_text(encoding='utf-8')
            configs[source] = json.loads(text)
            sources[source] = _key_locations(text)
        except (OSError,ValueError,IndexError,TypeError):
            continue
    output = ['[参数错误] '+str(exc),'现场配置：'+str(path),'默认模板：'+str(template)]
    if isinstance(exc,json.JSONDecodeError):
        output.append('JSON格式：行{}，列{}'.format(exc.lineno,exc.colno))
    tokens = set(re.findall(r'[A-Za-z_][A-Za-z_0-9]*(?:\.[A-Za-z_][A-Za-z_0-9]*)*',str(exc)))
    matches = {}
    for token in sorted(tokens):
        for source,cfg in configs.items():
            candidates = [token]
            if token.split('.')[0] in ('surface','shelf','ground'):
                candidates.append('actions.profiles.'+token)
            for index,point in enumerate(cfg.get('detection_points',[])):
                if not isinstance(point,dict):
                    continue
                pid = point.get('id')
                if token==pid:
                    candidates += ['detection_points[{}].{}'.format(index,key)
                                   for key in ('pose','detect_pose','layers','grasp_profile','difficulty')]
                elif pid and token.startswith(pid+'.'):
                    candidates.append('detection_points[{}].{}'.format(index,token[len(pid)+1:]))
            located = [(actual,line) for actual,line in sources[source]
                       if actual in candidates or ('.' not in token and actual.split('.')[-1]==token)]
            if located:
                for actual,line in located:
                    matches.setdefault(actual,(source,line))
                break  # 同一逻辑键在local找到就不报告模板中另一个索引的点。
    for actual,(source,line) in sorted(matches.items()):
        output.append('参数 {}：{}，第{}行'.format(actual,source,line))
    if not matches:
        output.append('缺失字段按上方报错键名补入现场配置；当前无法确定具体行号。')
    return '\n'.join(output)


def software_error(evidence, stage, exc):
    evidence.event('software_recovery',stage=stage,error_type=type(exc).__name__,
                   error=str(exc),traceback=traceback.format_exc())
    print('[软件恢复] {}：{}: {}'.format(stage,type(exc).__name__,exc),flush=True)
