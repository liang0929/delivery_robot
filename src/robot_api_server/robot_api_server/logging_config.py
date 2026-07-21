"""
統一 Logging 配置

使用方式：
    from .logging_config import setup_logging, get_logger
    setup_logging()
    logger = get_logger(__name__)
"""
import logging
import logging.handlers
import os
import sys
from pathlib import Path
from typing import Optional

# 日誌目錄
LOG_DIR = Path(os.environ.get('ROBOT_LOG_DIR', '/home/robot0/base_dev/logs'))
LOG_DIR.mkdir(parents=True, exist_ok=True)

# 日誌級別（可從環境變數設定）
LOG_LEVEL = os.environ.get('ROBOT_LOG_LEVEL', 'INFO').upper()

# 日誌格式
CONSOLE_FORMAT = '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s'
FILE_FORMAT = '%(asctime)s | %(levelname)-8s | %(name)s | %(funcName)s:%(lineno)d | %(message)s'
DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# ANSI 顏色碼
COLORS = {
    'DEBUG': '\033[36m',     # Cyan
    'INFO': '\033[32m',      # Green
    'WARNING': '\033[33m',   # Yellow
    'ERROR': '\033[31m',     # Red
    'CRITICAL': '\033[35m',  # Magenta
    'RESET': '\033[0m'
}


class ColoredFormatter(logging.Formatter):
    """彩色終端輸出 Formatter"""

    def format(self, record):
        # 保存原始 levelname
        orig_levelname = record.levelname
        # 加上顏色
        color = COLORS.get(record.levelname, '')
        reset = COLORS['RESET']
        record.levelname = f"{color}{record.levelname}{reset}"
        result = super().format(record)
        # 還原
        record.levelname = orig_levelname
        return result


def setup_logging(
    level: Optional[str] = None,
    log_file: Optional[str] = None,
    enable_file_logging: bool = True,
    enable_color: bool = True
) -> None:
    """
    設定全域 logging

    Args:
        level: 日誌級別 (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: 日誌檔案名稱 (預設: robot_api.log)
        enable_file_logging: 是否啟用檔案日誌
        enable_color: 是否啟用彩色輸出
    """
    level = level or LOG_LEVEL
    log_file = log_file or 'robot_api.log'

    # 取得 root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, level))

    # 清除現有 handlers
    root_logger.handlers.clear()

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(getattr(logging, level))

    if enable_color and sys.stdout.isatty():
        console_formatter = ColoredFormatter(CONSOLE_FORMAT, datefmt=DATE_FORMAT)
    else:
        console_formatter = logging.Formatter(CONSOLE_FORMAT, datefmt=DATE_FORMAT)

    console_handler.setFormatter(console_formatter)
    root_logger.addHandler(console_handler)

    # File handler (rotating)
    if enable_file_logging:
        log_path = LOG_DIR / log_file
        file_handler = logging.handlers.RotatingFileHandler(
            log_path,
            maxBytes=10 * 1024 * 1024,  # 10MB
            backupCount=5,
            encoding='utf-8'
        )
        file_handler.setLevel(getattr(logging, level))
        file_formatter = logging.Formatter(FILE_FORMAT, datefmt=DATE_FORMAT)
        file_handler.setFormatter(file_formatter)
        root_logger.addHandler(file_handler)

    # 降低第三方庫的日誌級別
    logging.getLogger('uvicorn').setLevel(logging.WARNING)
    logging.getLogger('uvicorn.access').setLevel(logging.WARNING)
    logging.getLogger('fastapi').setLevel(logging.WARNING)
    logging.getLogger('httpx').setLevel(logging.WARNING)
    logging.getLogger('httpcore').setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    取得 logger 實例

    Args:
        name: Logger 名稱，通常使用 __name__

    Returns:
        logging.Logger 實例
    """
    return logging.getLogger(name)


# 預設導出
__all__ = ['setup_logging', 'get_logger', 'LOG_DIR', 'LOG_LEVEL']
