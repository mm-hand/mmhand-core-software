# Generate the bilingual AI code guide

This is a procedure for a coding harness (Codex, GLM in another harness, or any
other assistant) to follow. The Python generator does **not** call a model. It
extracts committed code, checks annotations, and renders the guide. This file
defines the procedure, not facts about MMHand's runtime behavior. Derive those
facts from the exported source packets only; do not use handwritten README or
other narrative documents as evidence.

## 1. Prepare source packets

From the MMHand repository root, with the sibling `mmhand_teleop` checkout
available, run:

```powershell
python docs/ai/build/make.py --branch main
```

Replace `main` with the branch being documented. `--base <ref>` overrides the
comparison baseline if needed. `--prepare` is an explicit synonym for the
default mode. The command analyzes committed MMHand code at that branch and
committed `mmhand_teleop` code at its local `main`. It does not publish pages.
It prints the location of an ignored task directory under
`docs/ai/.explain_cache/tasks/`. Read its `index.json`, then each listed graph
packet. The packet includes both commits, a `bundle_hash`, the complete relevant
source functions or fragments with line numbers, source evidence IDs, the graph
manifest, and the exact `required_ids`. A `readme_commands` packet contains the
README operation commands. Read a parent graph before a child graph when
explaining their relationship; each graph packet still supplies its own source.

### Review and update the source scope first

The task directory also contains `scope_candidates.json`. It lists paths,
source hashes, changed ranges and symbol hints **without** copying every file's
body into the prompt. Full audits include already mapped paths and generated
source, each labeled by review status; only `unresolved` entries block
publication, and the harness must still review the whole list before stamping
the audit. The maintained path selection is
`docs/ai/source_scope.json`. Its `paths`, `entrypoints`, task lists and
`command_sources` replace fixed source paths in the extractor. Edit this JSON
when a configured entrypoint moves or is renamed; the selected path and symbol
must exist in the two committed revisions recorded by the candidate report.

For every candidate whose `status` is `unresolved`, inspect the committed source
and choose one disposition:

- Add a source-backed graph extension to `extensions`. Use `graph_id`, `repo`,
  `path`, `selector` (`function`, `definition`, or `file`), an existing
  `parent_id`, `relation`, and an `anchor` with `repo`, `path`, and exact
  `needle`. `function` and `definition` selectors must resolve uniquely;
  full-file selectors are restricted to configuration or script files. A
  direct `calls` edge is marked verified only when the anchor contains the
  selected function call **and** lies inside the parent node's cited source
  range. Other relationships remain visibly unverified until their complete
  flow can be proved.
- If the file is unrelated to the guide, add an `exclusions` entry with its
  `repo`, `path`, exact candidate `source_hash`, and a concrete `reason` of at
  least 20 characters. An exclusion expires when nonblank source content
  changes; a shift in line numbers alone does not expire it.

An extension looks like this (replace all example names with actual committed
source and an existing graph node):

```json
{
  "graph_id": "control",
  "repo": "mmhand",
  "path": "desktop/core/new_handler.py",
  "selector": {"kind": "function", "symbol": "handle_request"},
  "parent_id": "step_1",
  "relation": "calls",
  "anchor": {
    "repo": "mmhand",
    "path": "desktop/ui/ui_control_commands.py",
    "needle": "handle_request("
  }
}
```

When all candidates have a disposition, copy `branch`, `mmhand`, `teleop`, and
`mode` from `scope_candidates.json` → `commits` into the scope JSON's `audit`
object. On `main`, a changed MMHand or teleop commit requires a full review of
the committed source inventory; repeating the same pair reuses decisions.
Other branches review changed files and their symbol references. Pass
`--full-scope-audit` to force a full review on another branch. The first full
review can be large: search candidate symbols and open only the files needed
to decide each entry. Do not treat a file as unrelated solely because its name
is unfamiliar.

Run `python docs/ai/build/make.py --branch main` again after editing the scope.
New paths and nodes then enter the graph packets and their `required_ids`.
Changes to relevant source or graph relationships update only affected bundle
hashes. Commit the reviewed scope JSON with the annotations; task packets stay
in the ignored local cache.

## 2. Write explanations

### Mandatory installation and usage content

Both languages' **installation / usage** section must explain the actual wiring
and Arduino configuration from committed source packets. Cover P4 **and** S3:

- GPIO: P4 host USB D−/D+, CAN TX/RX and all four servo UART RX/TX pairs from
  `ESP32-P4/src/config/BoardConfig.h`; S3 encoder/tactile SPI, selector and CAN
  pins from `ESP32-S3/src/config/Config.h`. Never infer wiring from an old README.
- Which Type-C connector to use: the expansion-board **USB Serial/JTAG**
  connector for P4 runtime communication and debug; the Waveshare **CH343**
  connector is only a migration/recovery upload path. Cite the board adapter
  source and `tools/debug/debug_env.ps1`.
- Arduino IDE labels and exact FQBN options, including `USBMode=hwcdc`,
  `CDCOnBoot=cdc`, `UploadMode=default`, `JTAGAdapter=builtin`. Derive each
  board's options separately; a missing option must be marked unverified.
- Debug compile and upload commands for both targets, matching BIN/ELF hashes,
  USB adapter serial selection, and breakpoint commands from
  `build_debug.ps1`, `upload_debug.ps1`, `start_openocd.ps1`, `gdb_batch.py`.
  Explain hardware breakpoints on S3, deleting breakpoints before detach,
  `gdb_memory_map disable`, and lease expiry while halted. Resuming execution
  never authorizes automatic motor motion.
- Current GUI navigation, the single `/mmhand/joint_angle_deg` ROS2 input
  (`Float32MultiArray`, exactly J00–J20 in degrees), and control parameter JSON
  location/default restoration and whole-revision P4 synchronization.
- PaXini maintenance: direct S3 USB, scan all five finger buses, per-sensor
  New ID 0/1/2 mapping to device IDs 1/2/3, known model independent of ID,
  readback and power-cycle persistence, and inability to distinguish duplicate
  IDs on one motor or tactile bus. A full three-address tactile bus cannot
  perform a cyclic swap without first freeing a reachable address.

These are mandatory explanation topics, not permission to treat this prose as
runtime evidence. Add their committed code to `command_sources` and source
packets. Missing evidence, a stale bundle, or omitted required content must
block validation/publication; never fill gaps with guessed pins or commands.
Do not publish generated README/HTML from this uncommitted refactor. Prepare
against the selected committed revision, annotate its packets, then validate
and publish using the workflow below.

For each packet, write exactly one UTF-8 JSON file at its `annotation_path`
(normally `docs/ai/annotations/<graph_id>.json`). These files are generated
artifacts to keep with the guide. Copy `schema_version`, `instruction_version`,
`graph_id`, and `bundle_hash` exactly from the packet. Put **every**
`required_ids` entry in `items` exactly once. Do not include `templated_ids`:
the renderer explains those unconditional structure edges from verified graph
relationships. You may draft a large graph in several steps, but finish its one
complete JSON file before validation.

The JSON shape is:

```json
{
  "schema_version": 1,
  "instruction_version": 2,
  "graph_id": "<packet graph_id>",
  "bundle_hash": "<packet bundle_hash>",
  "overview": {
    "zh": "<source-backed graph overview in Chinese>",
    "en": "<source-backed graph overview in English>",
    "evidence_ids": ["<packet evidence id>"]
  },
  "glossary": [],
  "items": [
    {
      "id": "<one required ID>",
      "zh": {
        "plain": "", "condition": "", "effect": "", "failure": "",
        "recovery": "", "unknown": "", "difference": "", "change": ""
      },
      "en": {
        "plain": "", "condition": "", "effect": "", "failure": "",
        "recovery": "", "unknown": "", "difference": "", "change": ""
      },
      "evidence_ids": ["<this item's own packet evidence id>"]
    }
  ]
}
```

For `readme_commands`, use an empty overview
`{"zh":"","en":"","evidence_ids":[]}` and an empty glossary. For every
other graph, write a meaningful bilingual overview with a source evidence ID.
Optional glossary entries have `name`, `zh`, `en`, and `evidence_ids`; `name`
must match a source-defined variable listed in the packet. Every item must cite
at least one of its **own** manifest evidence IDs. It may cite other IDs from
the same graph to explain a relationship. Evidence IDs are the packet's stable
`src_...` identifiers, not README reference numbers or line numbers.

Explain each item for a leader unfamiliar with the code:

- `plain`: what it is and why it matters in this graph.
- `condition`: interpret each source variable and exact logical expression;
  state when entry or transition is allowed. Keep the raw expression visible.
- `effect`: what code actually writes, sends, calls, or changes next.
- `failure`: refusal, false branch, fault, alarm, or timeout outcome. For each
  conditional edge, state what happens when the condition is **true** and when
  it is **false**; if the false path is not proved, say so explicitly.
- `recovery`: the source-backed way back, if one is shown.
- `unknown`: what the packet does not establish. For safety rules, a catalog
  entry and call index alone do not prove the runtime trigger predicate.
- `difference`: how a command or mode differs from nearby alternatives.
- `change`: impact of a source change when the manifest marks this item changed.

Use substantial Chinese and English prose. Preserve variable names, original
conditions, graph IDs, and evidence IDs across both languages. Never infer
hardware observations from static code. Write `未核实` / `unverified` when a
claimed trigger, outcome, timing, or recovery is unsupported. Do not introduce
identifiers absent from the packet's source and manifest. The validator checks
structure, cited source IDs, minimum explanation length, original conditions,
and a list of known generic filler phrases. The harness remains responsible for
whether its explanation follows the cited code.

### Explanation quality gate

The eight fields must each contain a concrete, bilingual explanation. Do not
copy the same fallback sentence into every node. For each node, name the actual
function, state, rule, command, data structure, or device role; explain what
the shown code does and why it matters in this graph. For each condition, keep
the exact source expression and explain each operand in ordinary language.
For each conditional edge, explain both outcomes explicitly in both languages:
what the true branch does and what the false branch does. If a branch or
recovery path is absent from the packet, say which behavior is unverified and
why. The effect field must describe a source-backed call, write, message, or
state change; do not paste source excerpts as a substitute for explaining them.

The validator rejects known filler such as “is a node in the … graph,” “图中的
一个节点,” or a source-excerpt dump with no explanation. It also rejects short
fields, missing raw conditions, and conditional edges that omit true/false
outcomes. A structurally valid JSON file is not sufficient: review every graph
against its packet before publishing.

As a minimum floor, every field must contain at least 12 characters, `plain`
must contain at least 22 Chinese characters and 35 English characters, and a
conditional item's `condition` must include every exact expression listed in
its manifest in addition to its plain-language interpretation. Treat these as
rejection thresholds, not target lengths; complicated safety and state-machine
items need longer explanations.

## 3. Validate and publish

```powershell
python docs/ai/build/make.py --branch main --validate
python docs/ai/build/make.py --branch main --publish
```

Use the same branch and optional `--base` as preparation. `--validate` checks
the source-scope audit and every annotation against freshly extracted committed
source, reporting unresolved candidate paths as well as missing or stale graph
IDs. It does not replace the six published README/HTML files.
Correct the listed JSON files and run it again. `--publish` repeats validation,
then renders both README languages and both languages of the behavior and
architecture HTML. If any annotation is missing, stale, or invalid, it leaves
the six old guide files in place and writes `coverage_report.failed.json`.

When the source relevant to a graph changes, preparation gives that graph a new
`bundle_hash`; rewrite only that graph's annotation. Moving source lines without
changing the relevant graph content can reuse the JSON: publishing refreshes
the line-numbered links from the current committed revision. The generator
never commits or pushes the outputs.

After a successful publish, copy the freshly rendered bilingual guides to the
repository root so they are visible at the repo entry point:

- `docs/ai/AI_README.zh.md` → repo root `AI_README.zh.md`
- `docs/ai/AI_README.en.md` → repo root `AI_README.en.md`

Copy the rendered files verbatim; do not hand-edit either the originals or the
copies. Repeat this copy after every future publish, and keep the root copies
in the same commit as the regenerated guide.
