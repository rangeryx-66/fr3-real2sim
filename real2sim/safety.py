"""Shared fail-closed safety checks for post-grasp skills."""
from __future__ import annotations


class UnsafePayload(RuntimeError):
    pass


def require_free_space_stable(result: dict) -> dict:
    """Accept only a real, retained grasp that is clear of table support."""
    support = result.get("final_support") or result.get("support") or {}
    flags = support.get("flags", result.get("flags", {}))
    category = support.get("category")
    stable = bool(
        result.get("success")
        and support.get("passed")
        and support.get("currently_clear")
        and flags.get("PICKED")
        and flags.get("RETAINED", True)
        and flags.get("CLEAR_TABLE")
        and category not in {"DROP", "CONTACT_LOSS", "CONTINUOUS_SLIP", "ROTATIONAL_INSTABILITY"}
    )
    if not stable:
        raise UnsafePayload(
            "ObjectScan/PayloadID require FREE_SPACE_STABLE; "
            f"success={result.get('success')} category={category} flags={flags}"
        )
    return support
