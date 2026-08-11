# PSFP on the LAN9662: what is there, and where it stops

Unlike RedBox, this is not a case of a model without silicon. The device answers with real
capacity, read from bridge `b0` component `c0`:

```
max-flow-meter-instances:     64
max-stream-filter-instances:  64
max-stream-gate-instances:    64   (supported-list-max 4)
stream-identity:              []   -- exists, empty, addressable
```

Sixty-four per-stream policers and sixty-four per-stream gates. A gate schedule acts on a traffic
class leaving a port; these act on the frames a filter selects, wherever they came from. That is a
sharper instrument, and it is the one thing on this bench that could protect a single flow rather
than a whole priority.

## Read the deviations before writing anything

Each module ships a `-dev.yang` beside it, and it is the difference between what the standard
models and what this build implements. Three separate attempts here failed on nodes the deviation
files had already marked `not-supported`:

| node | consequence |
|---|---|
| `stream-handle-spec/wildcard` | a filter cannot match "any stream"; a real stream handle is required, so a stream identity must exist first |
| `flow-meter-enable` | must be omitted; referencing the meter is what enables it |
| `stream-identity/in-facing` | the identity has to be placed `out-facing` |
| `.../identification-type` | must be omitted from every identification method |
| stream gate `operation-name`, `config-change-error`, `oper-base-time` | absent |

The device's own errors said the same things afterwards — `SID not found` with the offending data
node number, which is precise and quick to look up. But the deviation file says it before the
round trip rather than after, and reading it first would have saved four of them.

## Where it stops: creating the list entries

Nothing was configured in the end. Every shape this device accepts elsewhere was rejected here,
each with a different message:

| shape | used successfully elsewhere for | here |
|---|---|---|
| list path, key inside the value | creating `interface` L3V1, adding an IP address | `lma_cc_append_keys:4119: Invalid SID` |
| entry path with key, leaves nested | — | `List keys not allowed` |
| leaf path with key | `interface[name='1']/enabled`, `gate-enabled` | `lma_cc_node_call_cb()` — the entry does not exist yet |

The third fails for a reason that makes sense: a leaf cannot be set on an entry that has not been
created. The first is the shape that creates entries everywhere else on this device, and it is the
one that has to be made to work. Whether the obstacle is the encoder in `keti-tsn-cli`, a
mandatory node missing from the payload, or the choice-and-case nesting (`parameters/
null-stream-identification/null-stream-identification/...`, where the name appears twice) is not
yet established — all three were tried, none succeeded.

Both tables read empty afterwards; the bench was left as it was found.

## What to try next

Ask the CLI to encode the payload without sending it (`keti-tsn encode`) and compare the CBOR
against a `get` of a device that already has an entry — the same technique that decoded the gate
schedule's container, which was written by hand from the encoder's own bytes and worked first
time. Failing that, MUP1 through the vendor's own `mvdct`, which presumably has a working shape
for these lists.
