# Lab records

Vision tuning sessions, one file per saved record. The diagnostics write them
to `/sdcard/FIRST/lab-records/` when gamepad 1 **A** is pressed after START;
`adb pull /sdcard/FIRST/lab-records/. lab-records/` copies them here.

Before committing a record, fill in its header (who, lighting, what was
verified, whether to adopt). Commit the Limelight pipeline file downloaded from
its web interface alongside the matching record.

A record is evidence, not configuration: nothing reads these files. Adopting
values means setting the config fields listed under `[adopt into the code]`
in a reviewed commit.
