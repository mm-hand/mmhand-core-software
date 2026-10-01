(()=>{"use strict";
const db=JSON.parse(document.getElementById("guide-data").textContent),
      graphs=Object.fromEntries(db.graphs.map(g=>[g.id,g])),t=db.labels,
      nav=document.getElementById("nav"),details=document.getElementById("details"),
      toolbar=document.getElementById("toolbar"),crumb=document.getElementById("breadcrumb");
let cy=null,current=null,search="",changedOnly=false,unknownOnly=false;
const esc=value=>String(value??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c]));
const label=value=>typeof value==="object"&&value!==null?(value[db.lang]??value.en??value.zh??""):String(value??"");
const h=(tag,content)=>`<${tag}>${content}</${tag}>`;
function sourceLinks(refs){if(!refs||!refs.length)return `<p class="empty">${esc(t.unknown)}</p>`;
 const seen=new Set();return refs.filter(r=>{const k=r.repo+":"+r.path+":"+r.start+":"+r.end;if(seen.has(k))return false;seen.add(k);return true}).map(r=>
 `<div><span class="chip${r.changed?" changed":""}">[${r.number}]</span><code>${esc(r.repo+":"+r.path)}:${r.start}-${r.end}</code><br><a target="_blank" rel="noopener" href="${esc(r.github)}">${esc(t.github)}</a><a href="${esc(r.local)}">${esc(t.local)}</a><pre>${esc(r.text)}</pre></div>`).join("");}
function factList(values){return `<ul>${values.map(v=>`<li>${esc(v)}</li>`).join("")}</ul>`;}
function relevant(g,node){return {incoming:g.edges.filter(e=>e.target===node.id),outgoing:g.edges.filter(e=>e.source===node.id)};}
function graphIntro(g){const summary=g.summary;if(!summary)return `<p class="empty">${esc(t.details)}</p>`;
 let out=`<h2>${esc(t.graphOverview)}</h2><section class="explanation"><p>${esc(summary.overview?.[db.lang]||"")}</p></section>`;
 if(summary.overview?.evidence?.length)out+=`<h3>${esc(t.explanationEvidence)}</h3>${sourceLinks(summary.overview.evidence)}`;
 if(summary.glossary?.length)out+=`<h3>${esc(t.graphGlossary)}</h3>`+summary.glossary.map(term=>
 `<div class="explanation"><strong><code>${esc(term.name)}</code></strong><p>${esc(term[db.lang])}</p>${sourceLinks(term.evidence)}</div>`).join("");
 return out;}
function showDetails(item,type){const g=graphs[current],d=item.details||{};let out=`<h2>${esc(label(item.label))}</h2>`;
 out+=`<p><span class="chip">${esc(item.kind||type)}</span>${item.changed?`<span class="chip changed">Δ</span>`:""}${item.unverified?`<span class="chip unknown">${esc(t.unknown)}</span>`:""}</p>`;
 const explained=item.explanation?.[db.lang];
 if(explained){const fields=[["plain",t.plain],["condition",t.conditionMeaning],["effect",t.effect],
 ["failure",t.failure],["recovery",t.recovery],["difference",t.difference],["change",t.changeImpact],["unknown",t.unknown]];
 out+=`<section class="explanation">`+fields.filter(([key])=>explained[key]?.trim()).map(([key,title])=>
 `<h3>${esc(title)}</h3><p>${esc(explained[key])}</p>`).join("")+`</section>`;}
 if(d.text)out+=h("p",esc(label(d.text)));
 for(const [key,value] of Object.entries(d)){if(["conditions","period","calls","text","chain"].includes(key)||value===null||value===undefined||value==="")continue;
 out+=`<p><strong>${esc(key)}:</strong> ${esc(typeof value==="object"&&("zh" in value||"en" in value)?label(value):typeof value==="object"?JSON.stringify(value):value)}</p>`;}
 if(item.condition)out+=`<h3>${esc(t.conditions)}</h3><pre>${esc(item.condition)}</pre>`;
 if(d.conditions?.length){out+=`<h3>${esc(t.conditions)}</h3>`+d.conditions.map(c=>`<div><pre>${esc(c.expr)}</pre>${sourceLinks([c.evidence])}</div>`).join("");}
 if(d.calls?.length)out+=`<h3>${esc(t.calls)}</h3>`+factList(d.calls);
 if(d.period?.length)out+=`<h3>${esc(t.period)}</h3>`+d.period.map(x=>typeof x==="string"?`<pre>${esc(x)}</pre>`:`<div><pre>${esc(x.expr)}</pre>${sourceLinks([x.evidence])}</div>`).join("");
 if(d.chain?.length)out+=`<h3>${esc(t.safetyChain)}</h3>`+d.chain.map(x=>
 `<div><strong>${esc(x.role)}</strong> · <code>${esc(x.path)}::${esc(x.symbol)}</code>${sourceLinks([x.evidence])}</div>`).join("");
 if(type==="node"){const paths=relevant(g,item);for(const [key,items] of Object.entries(paths))if(items.length){out+=`<h3>${esc(t[key])}</h3><ul>`+items.map(e=>`<li>${esc(e.source)} → ${esc(e.target)} · ${esc(label(e.label))}${e.condition?` · <code>${esc(e.condition)}</code>`:""}</li>`).join("")+"</ul>";}}
 if(item.drilldown&&graphs[item.drilldown])out+=`<button class="drill" id="drill">${esc(label(graphs[item.drilldown].title))} →</button>`;
 if(item.explanation?.evidence_ids?.length){const ids=new Set(item.explanation.evidence_ids);
 const cited=(item.evidence||[]).filter(r=>ids.has(`${r.repo}:${r.path}:L${r.start}-L${r.end}`));
 out+=`<h3>${esc(t.explanationEvidence)}</h3>${sourceLinks(cited)}`;}
 out+=`<h3>${esc(t.references)}</h3>${sourceLinks(item.evidence)}`;details.innerHTML=out;
 if(item.drilldown&&graphs[item.drilldown])document.getElementById("drill").onclick=()=>show(item.drilldown);
}
function renderNav(){nav.innerHTML="";const visible=db.graphs.filter(g=>!search||label(g.title).toLowerCase().includes(search)||g.nodes.some(n=>label(n.label).toLowerCase().includes(search)||JSON.stringify(n.details||{}).toLowerCase().includes(search)));
 for(const g of visible){const b=document.createElement("button");b.textContent=label(g.title);b.className=(g.parent?"child ":"")+(current===g.id?"active":"");b.onclick=()=>show(g.id);nav.append(b);}
 if(!visible.length)nav.innerHTML=`<div class="empty">${esc(t.unknown)}</div>`;}
function renderCrumb(g){crumb.innerHTML="";if(g.parent&&graphs[g.parent]){const back=document.createElement("button");back.textContent="← "+label(graphs[g.parent].title);back.onclick=()=>show(g.parent);crumb.append(back);}const here=document.createElement("strong");here.textContent=label(g.title);crumb.append(here);}
function show(id){const g=graphs[id];if(!g)return;current=id;location.hash=encodeURIComponent(id);renderNav();renderCrumb(g);details.innerHTML=graphIntro(g);
 const keep=n=>(!changedOnly||n.changed||n.kind==="group")&&(!unknownOnly||n.unverified||n.kind==="group");
 const nodes=g.nodes.filter(keep);const ids=new Set(nodes.map(n=>n.id));const edges=g.edges.filter(e=>ids.has(e.source)&&ids.has(e.target));
 const elements=[...nodes.map(n=>({data:{id:n.id,label:label(n.label),raw:n,kind:n.kind||"process",changed:!!n.changed,unverified:!!n.unverified}})),
                 ...edges.map((e,i)=>({data:{id:"edge_"+i,source:e.source,target:e.target,label:label(e.label),raw:e,changed:!!e.changed}}))];
 if(cy)cy.destroy();cy=cytoscape({container:document.getElementById("canvas"),elements,
 style:[{selector:"node",style:{"label":"data(label)","background-color":"#2b789d","color":"#fff","text-wrap":"wrap","text-max-width":"170px","font-size":"11px","text-valign":"center","shape":"round-rectangle","width":"label","height":"label","padding":"13px"}},
 {selector:'node[kind="state"]',style:{"background-color":"#235e8c"}},{selector:'node[kind="fault"]',style:{"background-color":"#99544a"}},
 {selector:'node[kind="group"]',style:{"background-color":"#365b6c"}},{selector:'node[changed = true]',style:{"border-width":4,"border-color":"#e5822e"}},
 {selector:'node[unverified = true]',style:{"border-width":3,"border-color":"#e1ba34"}},
 {selector:"edge",style:{"width":2,"line-color":"#91aebd","target-arrow-shape":"triangle","target-arrow-color":"#91aebd","curve-style":"bezier","label":"data(label)","font-size":"9px","text-background-color":"#fff","text-background-opacity":.95}},
 {selector:'edge[changed = true]',style:{"line-color":"#e5822e","target-arrow-color":"#e5822e","width":4}}],
 layout:{name:"breadthfirst",directed:true,padding:35,spacingFactor:1.35},wheelSensitivity:.16});
 cy.on("tap","node",ev=>showDetails(ev.target.data("raw"),"node"));cy.on("tap","edge",ev=>showDetails(ev.target.data("raw"),"edge"));
 cy.on("dbltap","node",ev=>{const child=ev.target.data("raw").drilldown;if(child)show(child)});
 if(search){cy.nodes().forEach(n=>{const match=n.data("label").toLowerCase().includes(search)||JSON.stringify(n.data("raw").details||{}).toLowerCase().includes(search);n.style("opacity",match?1:.28)});}
}
toolbar.innerHTML=`<input id="search" aria-label="${esc(t.search)}" placeholder="${esc(t.search)}"><label><input id="changed" type="checkbox"> ${esc(t.changed)}</label><label><input id="unknown" type="checkbox"> ${esc(t.unknown)}</label><p class="count">${db.graphs.length} graphs · ${db.meta.coverage.p4_states||0} states · ${db.meta.coverage.safety_rules||0} rules</p>`;
document.getElementById("search").oninput=e=>{search=e.target.value.trim().toLowerCase();renderNav();if(current)show(current)};
document.getElementById("changed").onchange=e=>{changedOnly=e.target.checked;show(current)};
document.getElementById("unknown").onchange=e=>{unknownOnly=e.target.checked;show(current)};
window.addEventListener("hashchange",()=>{const id=decodeURIComponent(location.hash.slice(1));if(graphs[id]&&id!==current)show(id)});
show(graphs[decodeURIComponent(location.hash.slice(1))]?decodeURIComponent(location.hash.slice(1)):(db.kind==="behavior"?"behavior_overview":"architecture_overview"));
})();
