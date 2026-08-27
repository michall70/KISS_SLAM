from pathlib import Path

import yaml

# 配置目录 = 本脚本所在目录
CONFIG_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = CONFIG_DIR / "config.yaml"


def _normalize(obj):
    """把 yaml 解析结果里的字符串 'None' 转成真正的 None。

    YAML 标准空值写作 null/~, 但有些人写 None(yaml 会解析成字符串 "None")。
    这里递归处理, 让 'None' 字符串在配置里也能表示空值。
    """
    if isinstance(obj, dict):
        return {k: _normalize(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_normalize(v) for v in obj]
    if isinstance(obj, str) and obj.strip() == "None":
        return None
    return obj


def load_config(path=None):
    """解析 yaml 配置文件 -> 嵌套字典。

    - path=None    : 用默认 config.yaml (脚本目录下)
    - 相对路径     : 相对脚本目录解析 (如 "mydata_config.yaml")
    - 绝对路径     : 直接用
    输入中的引号(' ")会被自动去除, 路径前后空白也会去掉。
    """
    if path is None:
        p = DEFAULT_CONFIG
    else:
        p = Path(str(path).strip().strip("'\""))
        if not p.is_absolute():
            p = CONFIG_DIR / p
    with open(p, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    return _normalize(raw)


if __name__ == "__main__":
    import pprint

    pprint.pprint(load_config())
