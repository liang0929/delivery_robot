"""規格 §6 的錯誤碼與回應格式。

回應體外層鍵名是 ``event``（不是 ``error``），這是規格明定的：

    { "event": { "code": "POINT_NOT_FOUND" } }
"""

from typing import Optional

from fastapi import Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

# --- 400 ---
INVALID_INPUT = "INVALID_INPUT"
GET_LOCATION_FAILED = "GET_LOCATION_FAILED"
MAP_MISMATCH = "MAP_MISMATCH"

# --- 404 ---
POINT_NOT_FOUND = "POINT_NOT_FOUND"
MAP_NOT_FOUND = "MAP_NOT_FOUND"
GROUP_NOT_FOUND = "GROUP_NOT_FOUND"
MISSING_POINT_NAME = "MISSING_POINT_NAME"
MISSING_VIRTUAL_WALL_NAME = "MISSING_VIRTUAL_WALL_NAME"
MISSING_GROUP_NAME = "MISSING_GROUP_NAME"
NOT_FOUND_IN_GROUP = "NOT_FOUND_IN_GROUP"

# --- 409 ---
ROBOT_BUSY = "ROBOT_BUSY"
NOT_IN_NAVIGATION_MODE = "NOT_IN_NAVIGATION_MODE"
POINT_NOT_IN_MAP = "POINT_NOT_IN_MAP"

# --- 500（規格未列，內部錯誤用）---
INTERNAL_ERROR = "INTERNAL_ERROR"

#: error code → 預設 HTTP 狀態碼
DEFAULT_STATUS = {
    INVALID_INPUT: 400,
    GET_LOCATION_FAILED: 400,
    MAP_MISMATCH: 400,
    POINT_NOT_FOUND: 404,
    MAP_NOT_FOUND: 404,
    GROUP_NOT_FOUND: 404,
    MISSING_POINT_NAME: 404,
    MISSING_VIRTUAL_WALL_NAME: 404,
    MISSING_GROUP_NAME: 404,
    NOT_FOUND_IN_GROUP: 404,
    ROBOT_BUSY: 409,
    NOT_IN_NAVIGATION_MODE: 409,
    POINT_NOT_IN_MAP: 409,
    INTERNAL_ERROR: 500,
}


class ApiError(Exception):
    """以規格錯誤碼拋出的例外。

    ``status_code`` 省略時依 :data:`DEFAULT_STATUS` 推導。
    """

    def __init__(self, code: str, status_code: Optional[int] = None):
        self.code = code
        self.status_code = status_code or DEFAULT_STATUS.get(code, 400)
        super().__init__(code)


def error_body(code: str) -> dict:
    """規格 §6.1 回應體"""
    return {"event": {"code": code}}


def register_exception_handlers(app) -> None:
    """把 FastAPI 預設的 ``{"detail": ...}`` 錯誤格式換成規格的 ``{"event": {...}}``"""

    @app.exception_handler(ApiError)
    async def _api_error_handler(request: Request, exc: ApiError):
        return JSONResponse(status_code=exc.status_code, content=error_body(exc.code))

    @app.exception_handler(RequestValidationError)
    async def _validation_handler(request: Request, exc: RequestValidationError):
        return JSONResponse(status_code=400, content=error_body(INVALID_INPUT))

    @app.exception_handler(StarletteHTTPException)
    async def _http_handler(request: Request, exc: StarletteHTTPException):
        # detail 若已是規格錯誤碼就直接沿用，否則依狀態碼給合理預設
        code = exc.detail if isinstance(exc.detail, str) and exc.detail in DEFAULT_STATUS else None
        if code is None:
            if exc.status_code == 404:
                code = MAP_NOT_FOUND
            elif exc.status_code == 409:
                code = ROBOT_BUSY
            elif exc.status_code >= 500:
                code = INTERNAL_ERROR
            else:
                code = INVALID_INPUT
        return JSONResponse(status_code=exc.status_code, content=error_body(code))
