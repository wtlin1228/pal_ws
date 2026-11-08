#!/usr/bin/env python3
"""Build the tutorial site: docs/*.md -> docs/site/*.html. Needs only `pip install markdown`.

    python3 docs/build.py        # then open docs/site/index.html (no server needed)

Pages are ordered by file name: index.md, ch00.md ... ch18.md, appendix.md, reading-list.md.
"""
import pathlib, re, shutil, markdown

DOCS = pathlib.Path(__file__).resolve().parent
SITE = DOCS / 'site'
PAGES = sorted(DOCS.glob('*.md'), key=lambda p: (p.name != 'index.md', p.name == 'reading-list.md', p.name))

TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<link rel="stylesheet" href="style.css">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.9.0/styles/github.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.9.0/highlight.min.js"></script>
</head><body>
<nav><a class="brand" href="index.html">ROS 2 Palletizing &amp; Depalletizing</a>{nav}</nav>
<main>{body}<footer>{prev}{next}</footer></main>
<script>if (window.hljs) hljs.highlightAll();</script>
</body></html>"""


def title_of(md_text, fallback):
    m = re.search(r'^#\s+(.+)$', md_text, re.M)
    return m.group(1).strip() if m else fallback


def main():
    SITE.mkdir(exist_ok=True)
    titles = {p: title_of(p.read_text(encoding='utf-8'), p.stem) for p in PAGES}
    nav_items = ''.join(f'<a href="{p.stem}.html">{titles[p]}</a>' for p in PAGES)
    for i, p in enumerate(PAGES):
        html = markdown.markdown(p.read_text(encoding='utf-8'),
                                 extensions=['fenced_code', 'tables', 'toc', 'sane_lists'],
                                 extension_configs={'toc': {'toc_depth': '2-3'}})
        prev = f'<a class="prev" href="{PAGES[i-1].stem}.html">&larr; {titles[PAGES[i-1]]}</a>' if i > 0 else ''
        nxt = f'<a class="next" href="{PAGES[i+1].stem}.html">{titles[PAGES[i+1]]} &rarr;</a>' if i + 1 < len(PAGES) else ''
        nav = nav_items.replace(f'href="{p.stem}.html">', f'href="{p.stem}.html" class="current">')
        (SITE / f'{p.stem}.html').write_text(TEMPLATE.format(title=titles[p], nav=nav, body=html, prev=prev, next=nxt), encoding='utf-8')
    (SITE / 'style.css').write_text((DOCS / 'style.css').read_text(encoding='utf-8'), encoding='utf-8')
    (SITE / 'img').mkdir(exist_ok=True)
    for svg in (DOCS / 'img').glob('*.svg'):
        shutil.copy(svg, SITE / 'img' / svg.name)
    print(f'built {len(PAGES)} pages into {SITE}')


if __name__ == '__main__':
    main()
