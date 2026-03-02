# Strategy-Control Error Codes v1.0

## 1. Purpose

Standardize rejection/failure semantics for strategy-control integration.

---

## 2. Codes

| Code | Stage | Meaning | Typical Handling |
|---|---|---|---|
| `OK` | SUBMIT/EXEC/DONE | Success path or accepted state | continue |
| `DUPLICATE_COMMAND_ID` | SUBMIT | command id already seen | regenerate command id |
| `INVALID_HEADER` | SUBMIT | malformed header (empty id, invalid time, invalid seq) | fix serialization/validation |
| `INVALID_PAYLOAD` | SUBMIT | malformed payload (self-docking, negative wait, invalid ids) | fix strategy payload |
| `FUTURE_STATE_SEQ` | SUBMIT | command built on future/unconfirmed state version | resync snapshot and resend |
| `EXPIRED` | SUBMIT/EXEC | command exceeded deadline | replan with fresh command |
| `QUEUE_FULL` | SUBMIT | runtime queue capacity exceeded | backoff and retry |
| `ARBITRATION_LOST` | EXEC | command loses conflict arbitration | replan or wait |
| `UNKNOWN_COMMAND` | DONE | completion requested for unknown inflight id | diagnose runtime mismatch |
| `EXECUTION_FAILED` | DONE | accepted command failed during execution | fallback/replan |
| `REVOKE_APPLIED` | SUBMIT | revoke request applied to queued commands | verify cancellation result |

---

## 3. Recommended Policy

1. Hard reject (`INVALID_*`, `DUPLICATE_*`) should not be auto-retried.
2. Temporal reject (`FUTURE_STATE_SEQ`, `EXPIRED`) should trigger state refresh and replan.
3. Runtime reject (`ARBITRATION_LOST`, `QUEUE_FULL`) can be retried with cooldown.
4. `EXECUTION_FAILED` must trigger explicit fallback policy.

---

## 4. Logging Contract

For every `REJECTED` or `FAILED` feedback:

1. log `command_id`, `stage`, `error_code`, `state_seq`, `t`.
2. preserve original command payload for replay/debug.

Version: `v1.0`

