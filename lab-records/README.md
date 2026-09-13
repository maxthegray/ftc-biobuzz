# Lab records

Vision tuning sessions, one file per saved record. The diagnostics write them
to `/sdcard/FIRST/lab-records/` when gamepad 1 **A** is pressed after START;
`make pull-lab-records` copies them here.

Before committing a record, fill in its header (who, lighting, what was
verified, whether to adopt). Commit the Limelight pipeline file downloaded from
its web interface alongside the matching record.

A record is evidence, not configuration: nothing reads these files. Adopting
values means editing the `DEFAULT_*` constants named under
`[adopt as compiled defaults]` in a reviewed commit.
