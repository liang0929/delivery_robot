"""編輯交易端點 🟢（規格 §5 Commit / Discard）。

- ``commit`` — 提交 points / virtual walls / groups 全部變更並套用
- ``discard`` — 捨棄全部未提交變更
"""

import asyncio

from fastapi import APIRouter, Response

from ..logging_config import get_logger
from ..store import store

logger = get_logger(__name__)

router = APIRouter(prefix="/edits", tags=["edits"])


@router.post("/commit")
async def commit() -> Response:
    """POST /v1/robot/edits/commit — 200"""
    written = await asyncio.to_thread(store.commit)
    logger.info(f"Committed edits for maps: {written or '(none)'}")
    return Response(status_code=200)


@router.post("/discard")
async def discard() -> Response:
    """POST /v1/robot/edits/discard — 200"""
    store.discard()
    logger.info("Discarded uncommitted edits")
    return Response(status_code=200)
