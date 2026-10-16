// thesis.md -> Thesis_draft.docx (docx-js). Run: NODE_PATH=<dir with node_modules> node build_docx.js
// Markup: # / ## / ### headings ("#*" = unnumbered), "- " bullets, "Table: caption" + "| a | b |" rows,
// "![caption](png){width in}" figures, "$$ eq $$ (n)" equations; inline *it* **bold** ~sub~ ^sup^ `code`.
const fs = require('fs'), path = require('path');
const d = require('docx');
const { Document, Packer, Paragraph, TextRun, ImageRun, Table, TableRow, TableCell, WidthType, AlignmentType, HeadingLevel,
        BorderStyle, ShadingType, LevelFormat, Footer, PageNumber, NumberFormat, TableOfContents, PageBreak } = d;

const DIR = __dirname, TEXT_W = 9070;                     // A4 with 2.5 cm margins, in DXA
const src = fs.readFileSync(path.join(DIR, 'thesis.md'), 'utf8').replace(/\r/g, '').split('\n');

function runs(text, base = {}) {
  const out = [];
  for (const part of text.split(/(\*\*[^*]+\*\*|\*[^*]+\*|~[^~]+~|\^[^^]+\^|`[^`]+`)/)) {
    if (!part) continue;
    if (/^\*\*.+\*\*$/.test(part)) out.push(new TextRun({ ...base, text: part.slice(2, -2), bold: true }));
    else if (/^\*.+\*$/.test(part)) out.push(new TextRun({ ...base, text: part.slice(1, -1), italics: true }));
    else if (/^~.+~$/.test(part)) out.push(new TextRun({ ...base, text: part.slice(1, -1), subScript: true }));
    else if (/^\^.+\^$/.test(part)) out.push(new TextRun({ ...base, text: part.slice(1, -1), superScript: true }));
    else if (/^`.+`$/.test(part)) out.push(new TextRun({ ...base, text: part.slice(1, -1), font: 'Consolas', size: base.size ? base.size - 2 : 20 }));
    else out.push(new TextRun({ ...base, text: part }));
  }
  return out;
}

function pngSize(file) {
  const b = fs.readFileSync(file);
  return { data: b, w: b.readUInt32BE(16), h: b.readUInt32BE(20) };
}

function figure(caption, file, inches) {
  const img = pngSize(path.join(DIR, file)), w = Math.round(inches * 96);
  return [
    new Paragraph({ alignment: AlignmentType.CENTER, keepNext: true, spacing: { before: 120, after: 60, line: 240 },
      children: [new ImageRun({ type: 'png', data: img.data, transformation: { width: w, height: Math.round(w * img.h / img.w) },
                                altText: { title: caption.split('. ')[0], description: caption.replace(/[*~^`]/g, ''), name: path.basename(file) } })] }),
    new Paragraph({ style: 'Caption', children: runs(caption) }),
  ];
}

const border = { style: BorderStyle.SINGLE, size: 4, color: 'A6A6A6' };
function table(caption, rows) {
  const cells = rows.map(r => r.replace(/^\||\|$/g, '').split('|').map(c => c.trim()));
  const n = cells[0].length, plain = s => (s || '').replace(/[*~^`]/g, '');
  // never narrower than the longest word (+ padding), long text capped so short numeric columns keep room
  const weight = [...Array(n)].map((_, j) => Math.max(
    2 + Math.max(...cells.flatMap(r => plain(r[j]).split(/\s+/).map(w => w.length))),
    plain(cells[0][j]).length / 2,
    Math.min(32, Math.max(...cells.slice(1).map(r => plain(r[j]).length)))));
  const sum = weight.reduce((a, b) => a + b, 0);
  const widths = weight.map(x => Math.floor(TEXT_W * x / sum)); widths[0] += TEXT_W - widths.reduce((a, b) => a + b, 0);
  const tbl = new Table({
    width: { size: TEXT_W, type: WidthType.DXA }, columnWidths: widths,
    rows: cells.map((r, i) => new TableRow({ tableHeader: i === 0, cantSplit: true, children: r.map((c, j) => new TableCell({
      width: { size: widths[j], type: WidthType.DXA }, borders: { top: border, bottom: border, left: border, right: border },
      shading: i === 0 ? { fill: 'EDEDED', type: ShadingType.CLEAR, color: 'auto' } : undefined,
      margins: { top: 40, bottom: 40, left: 90, right: 90 },
      children: [new Paragraph({ spacing: { before: 0, after: 0, line: 240 }, alignment: AlignmentType.LEFT,
                                 children: runs(c, { size: 19, bold: i === 0 }) })] })) })),
  });
  return [new Paragraph({ style: 'Caption', keepNext: true, spacing: { before: 200, after: 80 }, children: runs(caption) }), tbl,
          new Paragraph({ spacing: { before: 0, after: 120 }, children: [] })];
}

// LaTeX subset -> Office Math: _ ^ {} \frac \sqrt \left( \right) \left[ \right] \cos \sin \max \operatorname{} \mathrm{} \text{}
// \cdot \times \, \; \quad \qquad; other characters (Greek, primes, operators) are taken literally.
const SPACES = { ',': ' ', ';': ' ', quad: ' ', qquad: '  ', cdot: '·', times: '×' };
function mathOf(s) {
  let i = 0;
  const word = () => { const m = s.slice(i).match(/^[a-zA-Z]+|^./); i += m[0].length; return m[0]; };
  const braced = () => { i++; const start = i; let depth = 1; while (depth) { if (s[i] === '{') depth++; if (s[i] === '}') depth--; i++; } return s.slice(start, i - 1); };
  const seq = stop => { const out = []; while (i < s.length && !stop()) out.push(...scripted()); return out; };
  // a script or argument is one atom or a {group}: in L_1^2 the ^2 belongs to L, not to the 1
  const group = () => { while (s[i] === ' ') i++; if (s[i] !== '{') return atom(); i++; const g = seq(() => s[i] === '}'); i++; return g; };
  function scripted() {
    const base = atom(); let sub, sup;
    while (s[i] === '_' || s[i] === '^') { const c = s[i++]; const g = group(); if (c === '_') sub = g; else sup = g; }
    if (sub && sup) return [new d.MathSubSuperScript({ children: base, subScript: sub, superScript: sup })];
    if (sub) return [new d.MathSubScript({ children: base, subScript: sub })];
    if (sup) return [new d.MathSuperScript({ children: base, superScript: sup })];
    return base;
  }
  function atom() {
    if (s[i] === ' ') { i++; return []; }
    if (s[i] === '{') { i++; const g = seq(() => s[i] === '}'); i++; return g; }
    if (/[0-9]/.test(s[i])) { const m = s.slice(i).match(/^[0-9.]+/)[0]; i += m.length; return [new d.MathRun(m)]; }
    if (s[i] !== '\\') return [new d.MathRun(s[i++])];
    i++; const cmd = word();
    if (cmd in SPACES) return [new d.MathRun(SPACES[cmd])];
    if (cmd === 'frac') { const num = group(); return [new d.MathFraction({ numerator: num, denominator: group() })]; }
    if (cmd === 'sqrt') return [new d.MathRadical({ children: group() })];
    if (cmd === 'mathrm') return [new d.MathRun({ text: braced(), style: 'plain' })];
    if (cmd === 'text') return [new d.MathRun({ text: braced(), normalText: true })];
    if (cmd === 'left') {
      const open = s[i++], body = seq(() => s.startsWith('\\right', i)); i += 7;
      return [open === '[' ? new d.MathSquareBrackets({ children: body }) : new d.MathRoundBrackets({ children: body })];
    }
    const name = cmd === 'operatorname' ? braced() : cmd;      // \cos, \sin, \max, \operatorname{atan2}
    if (!['cos', 'sin', 'max', 'atan2'].includes(name)) throw new Error(`unknown math command \\${cmd} in: ${s}`);
    return [new d.MathFunction({ name: [new d.MathRun({ text: name, style: 'plain' })], children: group() })];
  }
  return new d.Math({ children: seq(() => false) });
}

// Borderless 3-cell row: the equation alone in its paragraph (so Word shows it in display size), number on the right.
const EQ_W = [1100, TEXT_W - 2200, 1100], none = { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' };
function equation(body, number) {
  const cell = (w, children, align) => new TableCell({ width: { size: w, type: WidthType.DXA }, verticalAlign: d.VerticalAlign.CENTER,
    borders: { top: none, bottom: none, left: none, right: none },
    children: [new Paragraph({ alignment: align, spacing: { before: 60, after: 60, line: 240 }, children })] });
  return [new Table({ width: { size: TEXT_W, type: WidthType.DXA }, columnWidths: EQ_W,
    borders: { top: none, bottom: none, left: none, right: none, insideHorizontal: none, insideVertical: none },
    rows: [new TableRow({ cantSplit: true, children: [cell(EQ_W[0], [], AlignmentType.LEFT),
      cell(EQ_W[1], [mathOf(body)], AlignmentType.CENTER), cell(EQ_W[2], [new TextRun(number)], AlignmentType.RIGHT)] })] }),
    new Paragraph({ spacing: { before: 0, after: 0, line: 120 }, children: [] })];
}

// Block parser: returns {front, main} paragraph lists (front = everything before the first numbered chapter).
const front = [], main = []; let out = front, i = 0;
while (i < src.length) {
  const line = src[i];
  if (!line.trim()) { i++; continue; }
  let m;
  if ((m = line.match(/^(#{1,3})(\*?) (.*)$/))) {
    const level = m[1].length, text = m[3];
    if (level === 1 && /^\d/.test(text)) out = main;
    if (level === 1 && text === 'Abstract') out.push(new Paragraph({ style: 'FrontTitle', children: [new TextRun(text)] }));
    else out.push(new Paragraph({ heading: [HeadingLevel.HEADING_1, HeadingLevel.HEADING_2, HeadingLevel.HEADING_3][level - 1],
                                  pageBreakBefore: level === 1 && out === main, children: [new TextRun(text)] }));
    if (level === 1 && text === 'Abstract') {                // abstract, then the table of contents
      i++; while (i < src.length && !src[i].startsWith('#')) { if (src[i].trim()) out.push(new Paragraph({ children: runs(src[i]) })); i++; }
      out.push(new Paragraph({ style: 'FrontTitle', pageBreakBefore: true, children: [new TextRun('Contents')] }));
      out.push(new TableOfContents('Contents', { hyperlink: true, headingStyleRange: '1-2' }));
      continue;
    }
    i++; continue;
  }
  if ((m = line.match(/^!\[(.*)\]\((.*)\)\{([\d.]+)\}$/))) { out.push(...figure(m[1], m[2], +m[3])); i++; continue; }
  if ((m = line.match(/^\$\$ (.*) \$\$ \((.*)\)$/))) { out.push(...equation(m[1], `(${m[2]})`)); i++; continue; }
  if ((m = line.match(/^Table: (.*)$/))) {
    const rows = []; i++; while (i < src.length && src[i].startsWith('|')) rows.push(src[i++]);
    out.push(...table(m[1], rows)); continue;
  }
  if (line.startsWith('- ')) { out.push(new Paragraph({ numbering: { reference: 'bullets', level: 0 }, spacing: { after: 60 }, children: runs(line.slice(2)) })); i++; continue; }
  if (/^\[\d+\] /.test(line)) { out.push(new Paragraph({ style: 'Reference', children: runs(line) })); i++; continue; }
  out.push(new Paragraph({ children: runs(line) })); i++;
}

const center = (text, size, extra = {}) => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 200 }, ...extra,
  children: [new TextRun({ text, size, ...(extra.run || {}) })] });
const titlePage = [
  center('', 24, { spacing: { before: 2400 } }),
  center('Design, Supervision and Experimental Evaluation of a Two-Link Planar Drawing Arm with Motor-Side Magnetic Encoders', 36, { run: { bold: true } }),
  center('Graduation Thesis', 28, { spacing: { before: 600, after: 1600 } }),
  center('Author: [Name]', 24), center('Student number: [Number]', 24), center('Supervisor: [Name]', 24),
  center('[School / Department]', 24), center('[University]', 24), center('October 2026', 24, { spacing: { before: 600 } }),
];

const footer = () => ({ default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ children: [PageNumber.CURRENT], size: 20 })] })] }) });
const page = { size: { width: 11906, height: 16838 }, margin: { top: 1418, bottom: 1418, left: 1418, right: 1418 } };

const doc = new Document({
  creator: 'Thesis draft', title: 'Two-link planar drawing arm', features: { updateFields: true },
  styles: {
    default: { document: { run: { font: 'Times New Roman', size: 24 } } },
    paragraphStyles: [
      { id: 'Normal', name: 'Normal', run: { font: 'Times New Roman', size: 24 }, paragraph: { spacing: { line: 360, after: 120 }, alignment: AlignmentType.JUSTIFIED } },
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 32, bold: true },
        paragraph: { spacing: { before: 0, after: 360, line: 276 }, alignment: AlignmentType.LEFT, keepNext: true, outlineLevel: 0 } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 28, bold: true },
        paragraph: { spacing: { before: 300, after: 120, line: 276 }, alignment: AlignmentType.LEFT, keepNext: true, outlineLevel: 1 } },
      { id: 'Heading3', name: 'Heading 3', basedOn: 'Normal', next: 'Normal', quickFormat: true, run: { size: 24, bold: true, italics: true },
        paragraph: { spacing: { before: 200, after: 80, line: 276 }, alignment: AlignmentType.LEFT, keepNext: true, outlineLevel: 2 } },
      { id: 'FrontTitle', name: 'Front Title', basedOn: 'Normal', next: 'Normal', run: { size: 32, bold: true },
        paragraph: { spacing: { before: 0, after: 360 }, alignment: AlignmentType.LEFT } },
      { id: 'Caption', name: 'Caption', basedOn: 'Normal', next: 'Normal', run: { size: 20 },
        paragraph: { spacing: { before: 60, after: 240, line: 240 }, alignment: AlignmentType.CENTER } },
      { id: 'Reference', name: 'Reference', basedOn: 'Normal', run: { size: 22 },
        paragraph: { spacing: { after: 100, line: 260 }, alignment: AlignmentType.LEFT, indent: { left: 567, hanging: 567 } } },
    ],
  },
  numbering: { config: [{ reference: 'bullets', levels: [{ level: 0, format: LevelFormat.BULLET, text: '•', alignment: AlignmentType.LEFT,
    style: { paragraph: { indent: { left: 567, hanging: 283 } } } }] }] },
  sections: [
    { properties: { page }, children: titlePage },
    { properties: { page: { ...page, pageNumbers: { start: 1, formatType: NumberFormat.LOWER_ROMAN } } }, footers: footer(), children: front },
    { properties: { page: { ...page, pageNumbers: { start: 1, formatType: NumberFormat.DECIMAL } } }, footers: footer(), children: main },
  ],
});

Packer.toBuffer(doc).then(buf => { fs.writeFileSync(path.join(DIR, 'Thesis_draft.docx'), buf); console.log('written', front.length, main.length); });
