#pragma once
// Page skeleton shared by the evaluation reports (light and dark themes).

#include <string>

inline std::string html_head(const std::string& title)
{
    return R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>)HTML" + title + R"HTML(</title><style>
:root{--bg:#f7f7f5;--plot:#ffffff;--ink:#1d1d1f;--muted:#8a8a8e;--grid:#e4e4e7;--left:#2563eb;--right:#eab308;--bad:#dc2626;--good:#16a34a;}
@media (prefers-color-scheme: dark){:root{--bg:#161618;--plot:#202024;--ink:#ececf0;--muted:#8a8a92;--grid:#2e2e33;--left:#4f8cff;--right:#facc15;--bad:#f87171;--good:#4ade80;}}
body{background:var(--bg);color:var(--ink);font:14px/1.45 -apple-system,system-ui,sans-serif;margin:0;padding:24px 16px;max-width:1300px;margin:auto}
h1{font-size:22px;margin:0 0 4px}h2{font-size:17px;margin:28px 0 8px}p{margin:4px 0 10px;color:var(--muted)}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:13px}
th,td{padding:4px 8px;border-bottom:1px solid var(--grid);text-align:right}th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){text-align:left}
.wrap{overflow-x:auto}.grid{display:flex;flex-wrap:wrap;gap:16px}
.card{background:var(--plot);border:1px solid var(--grid);border-radius:8px;padding:10px;max-width:100%}
.card svg{max-width:100%;height:auto;display:block}.card h3{font-size:14px;margin:0}.card .sub{font-size:12px;color:var(--muted);margin:2px 0 8px;max-width:420px}
.ok{color:var(--good)}.ko{color:var(--bad)}.id{font-family:ui-monospace,monospace;color:var(--muted)}
.legend span{margin-right:14px}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;vertical-align:middle;margin-right:4px;border:1px solid var(--ink)}
</style></head><body>
)HTML";
}
