import { readFileSync, existsSync, readdirSync } from "node:fs";
import { resolve, dirname, extname } from "node:path";
import { Marked } from "marked";
import puppeteer from "puppeteer-core";

const [,, mdPath, outPdf] = process.argv;
if (!mdPath || !outPdf) {
  console.error("usage: node render.mjs <input.md> <output.pdf>");
  process.exit(2);
}
const mdDir = dirname(resolve(mdPath));
const md = readFileSync(mdPath, "utf-8");

const mimeByExt = { ".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".gif": "image/gif", ".svg": "image/svg+xml" };

const escapeHtml = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

const marked = new Marked();
marked.use({
  renderer: {
    code({ text, lang }) {
      if (lang === "mermaid") {
        return `<pre class="mermaid">${escapeHtml(text)}</pre>`;
      }
      return `<pre><code>${escapeHtml(text)}</code></pre>`;
    },
    image({ href, title, text }) {
      let src = href;
      if (!/^(https?|data):/.test(href)) {
        const p = resolve(mdDir, href);
        if (existsSync(p)) {
          const mime = mimeByExt[extname(p).toLowerCase()] || "application/octet-stream";
          src = `data:${mime};base64,${readFileSync(p).toString("base64")}`;
        } else {
          console.error(`WARN missing image: ${href}`);
        }
      }
      const cap = text ? `<div class="img-caption">${text}</div>` : "";
      return `<figure><img src="${src}" alt="${text || ""}"/>${cap}</figure>`;
    },
  },
});

const body = marked.parse(md);
const html = `<!DOCTYPE html><html><head><meta charset="utf-8">
<style>
  body { font-family: "Noto Sans Mono CJK SC", "DejaVu Sans", sans-serif; font-size: 10.5pt; line-height: 1.65; color: #1a1a1a; max-width: 100%; }
  h1 { font-size: 20pt; border-bottom: 3px solid #2b6cb0; padding-bottom: 6px; }
  h2 { font-size: 15pt; color: #2b6cb0; border-bottom: 1px solid #cbd5e0; padding-top: 14px; padding-bottom: 4px; page-break-after: avoid; }
  h3 { font-size: 12.5pt; color: #2c5282; page-break-after: avoid; }
  pre { background: #f5f7fa; border: 1px solid #dde3ea; border-radius: 5px; padding: 9px 12px; font-size: 8.6pt; line-height: 1.45; overflow-x: hidden; white-space: pre-wrap; word-break: break-all; page-break-inside: avoid; }
  code { font-family: "Noto Sans Mono CJK SC", "DejaVu Sans Mono", monospace; }
  p code, li code, td code { background: #eef1f5; border-radius: 3px; padding: 0 3px; font-size: 9.3pt; }
  table { border-collapse: collapse; width: 100%; font-size: 9.3pt; margin: 10px 0; page-break-inside: avoid; }
  th, td { border: 1px solid #c4ccd4; padding: 5px 8px; text-align: left; }
  th { background: #edf2f7; }
  blockquote { border-left: 4px solid #2b6cb0; background: #f0f6fc; margin: 10px 0; padding: 6px 14px; color: #2d3748; }
  figure { margin: 14px 0; text-align: center; page-break-inside: avoid; }
  figure img { max-width: 96%; width: auto; height: auto; image-rendering: -webkit-optimize-contrast; border: 1px solid #d0d7de; border-radius: 4px; }
  .img-caption { font-size: 9pt; color: #57606a; margin-top: 5px; }
  hr { border: none; border-top: 1px dashed #a0aec0; margin: 22px 0; }
  a { color: #2b6cb0; text-decoration: none; }
  li { margin: 2px 0; }
  pre.mermaid { background: #fcfdfe; border: 1px solid #dde3ea; text-align: center; page-break-inside: avoid; }
  pre.mermaid svg { max-width: 96%; max-height: 580px; height: auto; }
</style></head><body>${body}</body></html>`;

const chromeDir = `${process.env.HOME}/.cache/puppeteer/chrome`;
const ver = readdirSync(chromeDir)[0];
const executablePath = `${chromeDir}/${ver}/chrome-linux64/chrome`;

const browser = await puppeteer.launch({ executablePath, args: ["--no-sandbox", "--disable-gpu"] });
const page = await browser.newPage();
await page.setViewport({ width: 1280, height: 1800, deviceScaleFactor: 2 });
// Large docs embed multi‑MB images as data URLs; networkidle0 often never settles.
await page.setContent(html, { waitUntil: "load", timeout: 180000 });

// Render mermaid diagrams in-page before printing.
const mermaidJs = readFileSync(resolve(import.meta.dirname, "node_modules/mermaid/dist/mermaid.min.js"), "utf-8");
await page.addScriptTag({ content: mermaidJs });
const mermaidReport = await page.evaluate(async () => {
  window.mermaid.initialize({
    startOnLoad: false,
    theme: "base",
    themeVariables: { fontFamily: '"Noto Sans Mono CJK SC", sans-serif', fontSize: "14px" },
    flowchart: { htmlLabels: true, curve: "linear" },
  });
  const nodes = [...document.querySelectorAll("pre.mermaid")];
  const errors = [];
  try {
    await window.mermaid.run({ querySelector: "pre.mermaid" });
  } catch (e) {
    errors.push(String(e && e.message ? e.message : e));
  }
  const rendered = nodes.filter((n) => n.querySelector("svg")).length;
  const failed = nodes.filter((n) => n.querySelector(".error-icon, [aria-roledescription='error'], .error") || /Syntax error/i.test(n.textContent || "")).length;
  const bare = nodes.filter((n) => !n.querySelector("svg")).length;
  return { total: nodes.length, rendered, failed, bare, errors };
});
console.log(`mermaid: ${JSON.stringify(mermaidReport)}`);
if (mermaidReport.bare > 0 || mermaidReport.failed > 0 || mermaidReport.errors.length) {
  console.error("ERROR: mermaid diagrams failed to render correctly");
  process.exit(1);
}
await new Promise((r) => setTimeout(r, 1500));
await page.pdf({
  path: outPdf,
  format: "A4",
  printBackground: true,
  margin: { top: "16mm", bottom: "16mm", left: "13mm", right: "13mm" },
  displayHeaderFooter: true,
  headerTemplate: "<div></div>",
  footerTemplate: `<div style="font-size:8px; width:100%; text-align:center; color:#888;"><span class="pageNumber"></span> / <span class="totalPages"></span></div>`,
});
await browser.close();
console.log(`PDF saved: ${outPdf}`);
