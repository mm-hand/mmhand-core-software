from pathlib import Path
import re

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path(__file__).resolve().parents[1]
MD_PATH = ROOT / "docs" / "UART_I2C_SPI_CAN通信协议入门教程.md"
OUT_PATH = ROOT / "docs" / "UART_I2C_SPI_CAN通信协议入门教程.docx"


def set_run_font(run, size=None, bold=None, color=None, name="Calibri", east_asia="Microsoft YaHei"):
    run.font.name = name
    run._element.rPr.rFonts.set(qn("w:eastAsia"), east_asia)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor.from_string(color)


def set_paragraph_format(paragraph, before=0, after=6, line=1.25):
    fmt = paragraph.paragraph_format
    fmt.space_before = Pt(before)
    fmt.space_after = Pt(after)
    fmt.line_spacing = line


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(table, top=80, start=120, bottom=80, end=120):
    tbl_pr = table._tbl.tblPr
    tbl_cell_mar = tbl_pr.find(qn("w:tblCellMar"))
    if tbl_cell_mar is None:
        tbl_cell_mar = OxmlElement("w:tblCellMar")
        tbl_pr.append(tbl_cell_mar)
    for m, v in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tbl_cell_mar.find(qn(f"w:{m}"))
        if node is None:
            node = OxmlElement(f"w:{m}")
            tbl_cell_mar.append(node)
        node.set(qn("w:w"), str(v))
        node.set(qn("w:type"), "dxa")


def set_table_width(table, widths):
    table.allow_autofit = False
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    tbl_pr = table._tbl.tblPr
    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), "9360")
    tbl_w.set(qn("w:type"), "dxa")
    tbl_ind = tbl_pr.find(qn("w:tblInd"))
    if tbl_ind is None:
        tbl_ind = OxmlElement("w:tblInd")
        tbl_pr.append(tbl_ind)
    tbl_ind.set(qn("w:w"), "120")
    tbl_ind.set(qn("w:type"), "dxa")
    for row in table.rows:
        for i, cell in enumerate(row.cells):
            width = widths[min(i, len(widths) - 1)]
            cell.width = Inches(width)
            tc_w = cell._tc.get_or_add_tcPr().find(qn("w:tcW"))
            if tc_w is None:
                tc_w = OxmlElement("w:tcW")
                cell._tc.get_or_add_tcPr().append(tc_w)
            tc_w.set(qn("w:w"), str(int(width * 1440)))
            tc_w.set(qn("w:type"), "dxa")


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = paragraph.add_run("第 ")
    set_run_font(run, 9, color="666666")
    fld_begin = OxmlElement("w:fldChar")
    fld_begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = "PAGE"
    fld_end = OxmlElement("w:fldChar")
    fld_end.set(qn("w:fldCharType"), "end")
    run._r.append(fld_begin)
    run._r.append(instr)
    run._r.append(fld_end)
    run2 = paragraph.add_run(" 页")
    set_run_font(run2, 9, color="666666")


def configure_document(doc):
    section = doc.sections[0]
    section.page_width = Inches(8.5)
    section.page_height = Inches(11)
    section.top_margin = Inches(1)
    section.bottom_margin = Inches(1)
    section.left_margin = Inches(1)
    section.right_margin = Inches(1)
    section.header_distance = Inches(0.492)
    section.footer_distance = Inches(0.492)

    styles = doc.styles
    normal = styles["Normal"]
    normal.font.name = "Calibri"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.font.size = Pt(11)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.25

    for name, size, color, before, after in (
        ("Heading 1", 16, "2E74B5", 18, 10),
        ("Heading 2", 13, "2E74B5", 14, 7),
        ("Heading 3", 12, "1F4D78", 10, 5),
    ):
        st = styles[name]
        st.font.name = "Calibri"
        st._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        st.font.size = Pt(size)
        st.font.color.rgb = RGBColor.from_string(color)
        st.font.bold = True
        st.paragraph_format.space_before = Pt(before)
        st.paragraph_format.space_after = Pt(after)
        st.paragraph_format.line_spacing = 1.25

    footer = section.footer.paragraphs[0]
    footer.text = "UART / I2C / SPI / CAN 通信协议入门教程"
    footer.alignment = WD_ALIGN_PARAGRAPH.LEFT
    for run in footer.runs:
        set_run_font(run, 9, color="666666")
    add_page_number(section.footer.add_paragraph())


def parse_inline(paragraph, text, size=11):
    # Minimal inline code and bold handling for this tutorial.
    token_re = re.compile(r"(`[^`]+`|\\*\\*[^*]+\\*\\*)")
    pos = 0
    for match in token_re.finditer(text):
        if match.start() > pos:
            run = paragraph.add_run(text[pos:match.start()])
            set_run_font(run, size)
        token = match.group(0)
        if token.startswith("`"):
            run = paragraph.add_run(token[1:-1])
            set_run_font(run, size, name="Consolas", east_asia="Microsoft YaHei")
            run.font.color.rgb = RGBColor.from_string("0B2545")
        elif token.startswith("**"):
            run = paragraph.add_run(token[2:-2])
            set_run_font(run, size, bold=True)
        pos = match.end()
    if pos < len(text):
        run = paragraph.add_run(text[pos:])
        set_run_font(run, size)


def add_code_block(doc, text):
    p = doc.add_paragraph()
    set_paragraph_format(p, before=2, after=8, line=1.0)
    p.paragraph_format.left_indent = Inches(0.18)
    for idx, line in enumerate(text.rstrip("\n").splitlines()):
        if idx:
            p.add_run().add_break()
        run = p.add_run(line)
        set_run_font(run, 9.5, name="Consolas", east_asia="Microsoft YaHei", color="0B2545")
    p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "F4F6F9")
    p._p.pPr.append(shd)


def add_markdown_table(doc, rows):
    data = []
    for row in rows:
        cells = [c.strip() for c in row.strip().strip("|").split("|")]
        data.append(cells)
    if len(data) < 2:
        return
    data = [data[0]] + data[2:]
    col_count = max(len(r) for r in data)
    table = doc.add_table(rows=len(data), cols=col_count)
    table.style = "Table Grid"
    set_cell_margins(table)
    if col_count == 2:
        widths = [1.7, 4.8]
    elif col_count == 3:
        widths = [1.5, 2.45, 2.55]
    elif col_count == 4:
        widths = [1.35, 1.85, 1.8, 1.5]
    else:
        widths = [6.5 / col_count] * col_count
    set_table_width(table, widths)

    for r_idx, row in enumerate(data):
        for c_idx in range(col_count):
            cell = table.cell(r_idx, c_idx)
            cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
            text = row[c_idx] if c_idx < len(row) else ""
            p = cell.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.LEFT
            set_paragraph_format(p, before=0, after=0, line=1.15)
            parse_inline(p, text, size=9.5)
            if r_idx == 0:
                set_cell_shading(cell, "E8EEF5")
                for run in p.runs:
                    run.bold = True
    doc.add_paragraph()


def add_picture(doc, md_dir, alt, rel_path):
    img_path = (md_dir / rel_path).resolve()
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_paragraph_format(p, before=6, after=3, line=1.0)
    run = p.add_run()
    run.add_picture(str(img_path), width=Inches(6.25))
    cap = doc.add_paragraph()
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_paragraph_format(cap, before=0, after=9, line=1.0)
    r = cap.add_run(alt)
    set_run_font(r, 9.5, color="666666")
    r.italic = True


def add_title_block(doc):
    p = doc.add_paragraph()
    set_paragraph_format(p, before=0, after=4, line=1.0)
    r = p.add_run("UART、I2C、SPI、CAN 通信协议入门教程")
    set_run_font(r, 24, bold=True, color="0B2545")

    p2 = doc.add_paragraph()
    set_paragraph_format(p2, before=0, after=12, line=1.2)
    r2 = p2.add_run("基于 ESP32-P4 Arduino 测试代码与 Kingst 实测/示意波形")
    set_run_font(r2, 11, color="666666")


def build():
    text = MD_PATH.read_text(encoding="utf-8")
    lines = text.splitlines()
    doc = Document()
    configure_document(doc)
    add_title_block(doc)

    md_dir = MD_PATH.parent
    in_code = False
    code_lines = []
    table_lines = []

    def flush_table():
        nonlocal table_lines
        if table_lines:
            add_markdown_table(doc, table_lines)
            table_lines = []

    for raw in lines[1:]:
        line = raw.rstrip()

        if line.startswith("```"):
            flush_table()
            if not in_code:
                in_code = True
                code_lines = []
            else:
                add_code_block(doc, "\n".join(code_lines))
                in_code = False
            continue

        if in_code:
            code_lines.append(line)
            continue

        if line.startswith("|") and line.endswith("|"):
            table_lines.append(line)
            continue
        flush_table()

        if not line.strip():
            continue

        img = re.match(r"!\[([^\]]*)\]\(([^)]+)\)", line)
        if img:
            add_picture(doc, md_dir, img.group(1), img.group(2))
            continue

        if line.startswith("## "):
            p = doc.add_paragraph(line[3:], style="Heading 1")
            continue
        if line.startswith("### "):
            p = doc.add_paragraph(line[4:], style="Heading 2")
            continue
        if line.startswith("# "):
            p = doc.add_paragraph(line[2:], style="Heading 1")
            continue
        if line.startswith("- "):
            p = doc.add_paragraph(style="List Bullet")
            set_paragraph_format(p, before=0, after=4, line=1.25)
            parse_inline(p, line[2:])
            continue
        m_num = re.match(r"^(\d+)\. (.+)$", line)
        if m_num:
            p = doc.add_paragraph(style="List Number")
            set_paragraph_format(p, before=0, after=4, line=1.25)
            parse_inline(p, m_num.group(2))
            continue

        p = doc.add_paragraph()
        set_paragraph_format(p, before=0, after=6, line=1.25)
        parse_inline(p, line)

    flush_table()
    doc.save(OUT_PATH)
    print(OUT_PATH)


if __name__ == "__main__":
    build()
