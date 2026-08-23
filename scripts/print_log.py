import sys

# ANSI 颜色码
COLOR = {
    "warn": "\033[93m",     # 黄色
    "info": "\033[94m",     # 蓝色
    "error": "\033[91m",    # 红色
    "success": "\033[92m",  # 绿色
}
RESET = "\033[0m"

LEVEL_UPPER = {
    "warn": "WARN",
    "info": "INFO",
    "error": "ERROR",
    "success": "SUCCESS"
}

def is_color_supported():
    """
    判断终端是否支持 ANSI 颜色。
    条件：是交互式终端且不是 Windows 旧版 cmd（这里假设 Linux/macOS 环境）
    """
    return sys.stdout.isatty()  # 简单判断，也可以加上对环境变量 TERM 的检查

def print_log(level: str, msg: str):
    """
    打印带颜色级别的日志，若终端不支持颜色则自动降级为纯文本。
    :param level: 'warn', 'info', 'error', 'success'
    :param msg:   日志内容
    """
    level_lower = level.lower()
    if level_lower not in LEVEL_UPPER:
        raise ValueError(f"不支持的日志级别: {level}，请使用 warn/info/error/success")

    level_upper = LEVEL_UPPER[level_lower]
    # 基础前缀
    prefix = f"[{level_upper}]"

    if is_color_supported():
        # 给级别部分上色，后面加一个空格隔开
        colored_prefix = f"{COLOR[level_lower]}{prefix}{RESET}"
        output = f"{colored_prefix} {msg}"
    else:
        output = f"{prefix} {msg}"

    try:
        print(output)
    except UnicodeEncodeError:
        # 极端情况降级（例如输出被重定向且编码非 UTF-8）
        print(f"{prefix} {msg}")

# ---------- 使用示例 ----------
if __name__ == "__main__":
    print_log("warn", "点云数量过多，已跳过部分")
    print_log("info", "加载配置文件成功")
    print_log("error", "bin file isn't exist")
    print_log("success", "地图保存完成")