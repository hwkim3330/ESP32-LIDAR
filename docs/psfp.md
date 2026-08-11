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

## Found: the CLI cannot express this entry

Encoding the payload without sending it settles it. A creation that works — an interface — comes
out as integer SIDs throughout:

```
a1 1907f1 a2 09 "L3V1" 181c 3162        {2033: {+9 name, +28 type}}
```

The stream identity, written with the choice nested as the YANG path suggests, comes out with a
**text string where a SID must be**:

```
a1 195dc5 a4 05 00 01 01 06 … 6a "parameters" …
                              ^^^^^^^^^^^^^^^^
```

`parameters` is a **choice**, and choices are transparent in CORECONF — they get no SID, and their
case's leaves are addressed as direct children of the parent. The encoder could not resolve it and
fell back to the name, which is exactly the `Invalid SID` the device complained about.

Flattening it removes the string. It also picks the wrong leaves:

```
a1 195dc5 a6 05 00 01 01 06 a1 01 81 "2" 195dd0 <mac> 195dd2 03 195dd3 01
                                     ^24016      ^24018   ^24019
```

24016, 24018 and 24019 are `dmac-vlan-stream-identification/down/{destination-mac,tagged,vlan}`.
The ones wanted are **24042, 24046, 24047** — the same three leaf names under
`null-stream-identification`. `destination-mac`, `tagged` and `vlan` appear in both cases of the
choice, the encoder takes the first match, and **`dmac-vlan-stream-identification` is exactly one
of the nodes this build marks `not-supported`**.

There is no way out of this from the YAML: null-stream-identification has no leaf that the other
cases do not also have, so nothing can disambiguate it by name.

## What to do about it

Write the CBOR by SID rather than by name — `{24005: {+5: 0, +1: 1, +6: {...}, +37: mac, +41:
tagged, +42: vlan}}`. The board already builds a gate schedule this way, from bytes checked
against this same encoder, and that one was byte-identical on the first try. `f<sid>` reads any
node already; a matching write is the missing half.

The alternative is the vendor's own `mvdct`, which presumably has a shape for these lists that
its own CLI lacks.
