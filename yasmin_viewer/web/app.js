/*
Copyright (C) 2026 Maik Knof

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//import mermaid from "https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs";

const state = {
  fsms: {},
  currentFsm: "ALL",
  hideNested: false,
  showOnlyActive: false,
  lastSuccessMs: 0,
  renderGeneration: 0,
  lastStructureSignature: "",
  lastActiveSignature: "",
  diagramCache: new Map(),
  viewportState: new Map(),
};

const fsmSelect = document.getElementById("fsm-select");
const hideNestedCheckbox = document.getElementById("hide-nested");
const showActiveCheckbox = document.getElementById("show-active");
const refreshButton = document.getElementById("refresh-button");
const emptyState = document.getElementById("empty-state");
const viewerPanel = document.getElementById("viewer-panel");
const viewerTitle = document.getElementById("viewer-title");
const viewerSubtitle = document.getElementById("viewer-subtitle");
const statusBadge = document.getElementById("status-badge");
const diagramList = document.getElementById("diagram-list");

mermaid.initialize({
  startOnLoad: false,
  theme: "base",
  securityLevel: "strict",
  state: {
    useMaxWidth: false,
  },
  themeVariables: {
    primaryColor: "#eff6ff",
    primaryBorderColor: "#334155",
    lineColor: "#475569",
    primaryTextColor: "#0f172a",
    fontFamily: "Arial, Helvetica, sans-serif",
    tertiaryColor: "#f8fafc",
  },
});

function stableStringify(value) {
  if (Array.isArray(value)) {
    return `[${value.map(stableStringify).join(",")}]`;
  }

  if (value && typeof value === "object") {
    const keys = Object.keys(value).sort();
    return `{${keys
      .map((key) => `${JSON.stringify(key)}:${stableStringify(value[key])}`)
      .join(",")}}`;
  }

  return JSON.stringify(value);
}

function stripActivityFromFsms(fsms) {
  const result = {};

  Object.entries(fsms).forEach(([fsmName, states]) => {
    result[fsmName] = Array.isArray(states)
      ? states.map((entry) => ({
          ...entry,
          current_state: 0,
        }))
      : [];
  });

  return result;
}

function getStructureSignature(fsms) {
  return stableStringify({
    fsms: stripActivityFromFsms(fsms),
    currentFsm: state.currentFsm,
    hideNested: state.hideNested,
    showOnlyActive: state.showOnlyActive,
  });
}

function getActiveSignature(fsms) {
  return stableStringify({
    fsms,
    currentFsm: state.currentFsm,
    hideNested: state.hideNested,
    showOnlyActive: state.showOnlyActive,
  });
}

function setStatus(text, cssClass) {
  statusBadge.textContent = text;
  statusBadge.className = `status-badge ${cssClass}`;
}

function clearElement(element) {
  while (element.firstChild) {
    element.removeChild(element.firstChild);
  }
}

function escapeMermaidLabel(value) {
  return String(value).replace(/"/g, '\\"');
}

function sanitizeId(value) {
  return String(value).replace(/[^a-zA-Z0-9_]/g, "_");
}

function updateFsmSelector() {
  const names = ["ALL", ...Object.keys(state.fsms).sort()];
  const previousValue = state.currentFsm;

  clearElement(fsmSelect);
  names.forEach((name) => {
    const option = document.createElement("option");
    option.value = name;
    option.textContent = name;
    fsmSelect.appendChild(option);
  });

  state.currentFsm = names.includes(previousValue) ? previousValue : "ALL";
  fsmSelect.value = state.currentFsm;
}

function getSelectedFsms() {
  const names = Object.keys(state.fsms).sort();
  return names
    .filter((name) => state.currentFsm === "ALL" || state.currentFsm === name)
    .map((name) => ({
      name,
      states: state.fsms[name],
    }))
    .filter(({ states }) => Array.isArray(states) && states.length > 0);
}

function cloneNodes(rawStates, fsmName, fsmIndex) {
  const nodesById = new Map();

  rawStates.forEach((entry) => {
    nodesById.set(entry.id, {
      ...entry,
      fsmName,
      fsmIndex,
      alias: `fsm_${fsmIndex}_state_${entry.id}`,
      active: false,
      hasActiveDescendant: false,
      isSyntheticOutcome: false,
    });
  });

  return nodesById;
}

function computeActiveFlags(nodesById) {
  nodesById.forEach((node) => {
    if (nodesById.has(node.current_state)) {
      nodesById.get(node.current_state).active = true;
    }
  });

  nodesById.forEach((node) => {
    if (!node.active) {
      return;
    }

    let parentId = node.parent;
    while (nodesById.has(parentId)) {
      const parent = nodesById.get(parentId);
      parent.hasActiveDescendant = true;
      parentId = parent.parent;
    }
  });
}

function buildChildrenMap(nodesById) {
  const childrenByParent = new Map();

  nodesById.forEach((node) => {
    if (!childrenByParent.has(node.parent)) {
      childrenByParent.set(node.parent, []);
    }
    childrenByParent.get(node.parent).push(node.id);
  });

  childrenByParent.forEach((ids) => {
    ids.sort((a, b) => a - b);
  });

  return childrenByParent;
}

function shouldKeepNode(node, nodesById) {
  if (node.parent === -1) {
    return true;
  }

  if (!state.showOnlyActive) {
    return true;
  }

  if (node.active || node.hasActiveDescendant) {
    return true;
  }

  let parentId = node.parent;
  while (nodesById.has(parentId)) {
    const parent = nodesById.get(parentId);
    if (parent.active || parent.hasActiveDescendant) {
      return true;
    }
    parentId = parent.parent;
  }

  return false;
}

function shouldExpandNode(node) {
  if (!node.is_fsm) {
    return false;
  }

  if (node.parent === -1) {
    return true;
  }

  return !state.hideNested;
}

function buildSiblingNameMap(nodesById) {
  const siblingNameMap = new Map();

  nodesById.forEach((node) => {
    const key = `${node.parent}::${node.name}`;
    siblingNameMap.set(key, node.id);
  });

  return siblingNameMap;
}

function prepareGraph(fsmName, rawStates, fsmIndex) {
  const nodesById = cloneNodes(rawStates, fsmName, fsmIndex);
  computeActiveFlags(nodesById);

  const visibleNodesById = new Map();
  nodesById.forEach((node, id) => {
    if (shouldKeepNode(node, nodesById)) {
      visibleNodesById.set(id, { ...node });
    }
  });

  const childrenByParent = buildChildrenMap(visibleNodesById);
  const siblingNameMap = buildSiblingNameMap(visibleNodesById);
  const syntheticOutcomes = new Map();

  function getOrCreateOutcomeNode(parentId, outcomeName) {
    const key = `${parentId}::${outcomeName}`;
    if (syntheticOutcomes.has(key)) {
      return syntheticOutcomes.get(key);
    }

    const node = {
      id: key,
      parent: parentId,
      name: outcomeName,
      alias: `fsm_${fsmIndex}_outcome_${sanitizeId(parentId)}_${sanitizeId(outcomeName)}`,
      is_fsm: false,
      current_state: -1,
      outcomes: [],
      transitions: [],
      active: false,
      hasActiveDescendant: false,
      isSyntheticOutcome: true,
    };

    syntheticOutcomes.set(key, node);
    return node;
  }

  visibleNodesById.forEach((node) => {
    node.resolvedTransitions = [];

    (node.transitions || []).forEach((transition) => {
      const outcomeLabel = transition[0];
      const targetName = transition[1];
      const targetKey = `${node.parent}::${targetName}`;

      if (siblingNameMap.has(targetKey)) {
        const targetId = siblingNameMap.get(targetKey);
        node.resolvedTransitions.push({
          outcome: outcomeLabel,
          targetAlias: visibleNodesById.get(targetId).alias,
        });
      } else {
        const outcomeNode = getOrCreateOutcomeNode(node.parent, targetName);
        node.resolvedTransitions.push({
          outcome: outcomeLabel,
          targetAlias: outcomeNode.alias,
        });
      }
    });
  });

  function renderScope(parentId, indent) {
    const lines = [];
    const childIds = childrenByParent.get(parentId) || [];
    const children = childIds
      .map((id) => visibleNodesById.get(id))
      .filter((node) => node !== undefined);

    const scopeOutcomeNodes = Array.from(syntheticOutcomes.values()).filter(
      (node) => node.parent === parentId,
    );

    children.forEach((node) => {
      const childHasVisibleChildren =
        (childrenByParent.get(node.id) || []).length > 0;

      if (shouldExpandNode(node) && childHasVisibleChildren) {
        lines.push(
          `${indent}state "${escapeMermaidLabel(node.name)}" as ${node.alias} {`,
        );
        lines.push(...renderScope(node.id, `${indent}  `));
        lines.push(`${indent}}`);
      } else {
        lines.push(
          `${indent}state "${escapeMermaidLabel(node.name)}" as ${node.alias}`,
        );
      }
    });

    scopeOutcomeNodes.forEach((node) => {
      lines.push(
        `${indent}state "${escapeMermaidLabel(node.name)}" as ${node.alias}`,
      );
    });

    children.forEach((node) => {
      node.resolvedTransitions.forEach((transition) => {
        lines.push(
          `${indent}${node.alias} --> ${transition.targetAlias} : ${escapeMermaidLabel(transition.outcome)}`,
        );
      });
    });

    return lines;
  }

  const rootNodes = (childrenByParent.get(-1) || [])
    .map((id) => visibleNodesById.get(id))
    .filter((node) => node !== undefined);

  const lines = [];
  lines.push("stateDiagram-v2");
  lines.push("  direction LR");
  lines.push(
    "  classDef outcome fill:#fff7ed,stroke:#c2410c,stroke-width:2px,color:#7c2d12",
  );

  rootNodes.forEach((rootNode) => {
    const rootChildren = (childrenByParent.get(rootNode.id) || []).length;
    const rootOutcomeChildren = Array.from(syntheticOutcomes.values()).filter(
      (node) => node.parent === rootNode.id,
    ).length;

    if (rootChildren > 0 || rootOutcomeChildren > 0) {
      lines.push(...renderScope(rootNode.id, "  "));
    } else {
      lines.push(
        `  state "${escapeMermaidLabel(rootNode.name)}" as ${rootNode.alias}`,
      );
    }
  });

  syntheticOutcomes.forEach((node) => {
    lines.push(`  class ${node.alias} outcome`);
  });

  return {
    definition: lines.join("\n"),
    visibleNodeCount: visibleNodesById.size,
    transitionCount: Array.from(visibleNodesById.values()).reduce(
      (sum, node) => sum + node.resolvedTransitions.length,
      0,
    ),
    nodesById,
    visibleNodesById,
    syntheticOutcomes,
  };
}

function removeInitialStateArtifacts(container) {
  const svg = container.querySelector("svg");
  if (!svg) {
    return;
  }

  const removedPoints = [];
  const candidateCircles = Array.from(svg.querySelectorAll("circle")).filter(
    (circle) => Number(circle.getAttribute("r") || "0") <= 10,
  );

  candidateCircles.forEach((circle) => {
    const cx = Number(circle.getAttribute("cx") || "0");
    const cy = Number(circle.getAttribute("cy") || "0");

    removedPoints.push({ x: cx, y: cy });

    const group = circle.closest("g");
    if (
      group &&
      group.querySelectorAll("circle").length === 1 &&
      group.querySelectorAll("text").length === 0
    ) {
      group.remove();
    } else {
      circle.remove();
    }
  });

  const hitRadius = 24;

  Array.from(svg.querySelectorAll("path")).forEach((path) => {
    if (typeof path.getTotalLength !== "function") {
      return;
    }

    try {
      const totalLength = path.getTotalLength();
      const start = path.getPointAtLength(0);
      const end = path.getPointAtLength(Math.max(0, totalLength - 0.1));

      const touchesRemovedPoint = removedPoints.some((point) => {
        const startDistance = Math.hypot(start.x - point.x, start.y - point.y);
        const endDistance = Math.hypot(end.x - point.x, end.y - point.y);
        return startDistance < hitRadius || endDistance < hitRadius;
      });

      if (!touchesRemovedPoint) {
        return;
      }

      const group = path.closest("g");
      if (
        group &&
        group.querySelectorAll("path").length === 1 &&
        group.querySelectorAll("text").length === 0
      ) {
        group.remove();
      } else {
        path.remove();
      }
    } catch (error) {
      console.error(error);
    }
  });
}

function getCacheKey(fsmName) {
  return JSON.stringify({
    fsmName,
    currentFsm: state.currentFsm,
    hideNested: state.hideNested,
    showOnlyActive: state.showOnlyActive,
  });
}

function getViewportKey(fsmName) {
  return `${fsmName}::${state.currentFsm}::${state.hideNested}::${state.showOnlyActive}`;
}

function getDefaultViewportState() {
  return {
    scrollLeft: 0,
    scrollTop: 0,
    zoom: 1,
  };
}

function getViewportState(viewportKey) {
  return state.viewportState.get(viewportKey) || getDefaultViewportState();
}

function saveViewportPosition(viewport, viewportKey, zoomWrapper) {
  state.viewportState.set(viewportKey, {
    scrollLeft: viewport.scrollLeft,
    scrollTop: viewport.scrollTop,
    zoom: Number(zoomWrapper.dataset.zoom || "1"),
  });
}

function restoreViewportPosition(viewport, viewportKey, zoomWrapper) {
  const saved = getViewportState(viewportKey);
  setZoom(zoomWrapper, saved.zoom);
  viewport.scrollLeft = saved.scrollLeft;
  viewport.scrollTop = saved.scrollTop;
}

function clampZoom(value) {
  return Math.min(3.0, Math.max(0.3, value));
}

function getZoom(zoomWrapper) {
  return Number(zoomWrapper.dataset.zoom || "1");
}

function setZoom(zoomWrapper, zoom) {
  const clampedZoom = clampZoom(zoom);
  zoomWrapper.dataset.zoom = String(clampedZoom);
  zoomWrapper.style.transform = `scale(${clampedZoom})`;
}

function zoomAroundPoint(viewport, zoomWrapper, nextZoom, clientX, clientY) {
  const previousZoom = getZoom(zoomWrapper);
  const clampedZoom = clampZoom(nextZoom);

  if (Math.abs(clampedZoom - previousZoom) < 1e-6) {
    return;
  }

  const rect = viewport.getBoundingClientRect();
  const offsetX = clientX - rect.left + viewport.scrollLeft;
  const offsetY = clientY - rect.top + viewport.scrollTop;
  const contentX = offsetX / previousZoom;
  const contentY = offsetY / previousZoom;

  setZoom(zoomWrapper, clampedZoom);

  viewport.scrollLeft = contentX * clampedZoom - (clientX - rect.left);
  viewport.scrollTop = contentY * clampedZoom - (clientY - rect.top);

  const viewportKey = viewport.dataset.viewportKey;
  if (viewportKey) {
    saveViewportPosition(viewport, viewportKey, zoomWrapper);
  }
}

function enableViewportInteraction(viewport, viewportKey, zoomWrapper) {
  let dragging = false;
  let pointerId = null;
  let startX = 0;
  let startY = 0;
  let startScrollLeft = 0;
  let startScrollTop = 0;
  let moved = false;

  viewport.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) {
      return;
    }

    dragging = true;
    pointerId = event.pointerId;
    startX = event.clientX;
    startY = event.clientY;
    startScrollLeft = viewport.scrollLeft;
    startScrollTop = viewport.scrollTop;
    moved = false;
    viewport.classList.add("dragging");
    viewport.setPointerCapture(pointerId);
  });

  viewport.addEventListener("pointermove", (event) => {
    if (!dragging || event.pointerId !== pointerId) {
      return;
    }

    const deltaX = event.clientX - startX;
    const deltaY = event.clientY - startY;

    if (Math.abs(deltaX) > 2 || Math.abs(deltaY) > 2) {
      moved = true;
    }

    viewport.scrollLeft = startScrollLeft - deltaX;
    viewport.scrollTop = startScrollTop - deltaY;
    saveViewportPosition(viewport, viewportKey, zoomWrapper);
  });

  function stopDragging(event) {
    if (!dragging) {
      return;
    }

    if (event && event.pointerId !== pointerId) {
      return;
    }

    dragging = false;
    pointerId = null;
    viewport.classList.remove("dragging");
    saveViewportPosition(viewport, viewportKey, zoomWrapper);
  }

  viewport.addEventListener("pointerup", (event) => {
    stopDragging(event);
  });

  viewport.addEventListener("pointercancel", (event) => {
    stopDragging(event);
  });

  viewport.addEventListener("pointerleave", (event) => {
    stopDragging(event);
  });

  viewport.addEventListener("click", (event) => {
    if (moved) {
      event.preventDefault();
      event.stopPropagation();
      moved = false;
    }
  });

  viewport.addEventListener("scroll", () => {
    saveViewportPosition(viewport, viewportKey, zoomWrapper);
  });

  viewport.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();

      const factor = event.deltaY < 0 ? 1.1 : 0.9;
      zoomAroundPoint(
        viewport,
        zoomWrapper,
        getZoom(zoomWrapper) * factor,
        event.clientX,
        event.clientY,
      );
    },
    { passive: false },
  );
}

function getMermaidNodeCandidates(svg, alias) {
  const safeAlias =
    typeof CSS !== "undefined" && typeof CSS.escape === "function"
      ? CSS.escape(alias)
      : alias;

  const selectors = [
    `g[id="${safeAlias}"]`,
    `g[id$="${safeAlias}"]`,
    `g[id*="${safeAlias}"]`,
    `g[class*="${safeAlias}"]`,
    `[id="${safeAlias}"]`,
    `[id$="${safeAlias}"]`,
    `[id*="${safeAlias}"]`,
    `[class*="${safeAlias}"]`,
  ];

  const result = [];
  selectors.forEach((selector) => {
    svg.querySelectorAll(selector).forEach((element) => {
      result.push(element);
    });
  });

  return result;
}

function getRenderableNodeGroup(candidate) {
  if (!candidate) {
    return null;
  }

  const directGroup = candidate.closest(
    "g.node, g.stateGroup, g.cluster, g.statediagram-state",
  );
  if (directGroup) {
    return directGroup;
  }

  let current = candidate;
  while (
    current &&
    current.tagName &&
    current.tagName.toLowerCase() !== "svg"
  ) {
    const hasShape =
      current.querySelector?.("rect, path, polygon, circle, ellipse") !== null;
    const hasText =
      current.querySelector?.("text, tspan, foreignObject") !== null;

    if (current.tagName.toLowerCase() === "g" && hasShape && hasText) {
      return current;
    }

    current = current.parentElement;
  }

  return candidate.closest("g");
}

function indexRenderedNodes(svg, graph) {
  const aliasToElement = new Map();

  graph.visibleNodesById.forEach((node) => {
    if (node.isSyntheticOutcome) {
      return;
    }

    const candidates = getMermaidNodeCandidates(svg, node.alias);
    for (const candidate of candidates) {
      const group = getRenderableNodeGroup(candidate);
      if (group) {
        aliasToElement.set(node.alias, group);
        group.dataset.stateAlias = node.alias;
        break;
      }
    }
  });

  graph.syntheticOutcomes.forEach((node) => {
    const candidates = getMermaidNodeCandidates(svg, node.alias);
    for (const candidate of candidates) {
      const group = getRenderableNodeGroup(candidate);
      if (group) {
        aliasToElement.set(node.alias, group);
        group.dataset.stateAlias = node.alias;
        group.classList.add("fsm-outcome-node");
        break;
      }
    }
  });

  return aliasToElement;
}

function clearHighlightClasses(root) {
  root.querySelectorAll(".fsm-active-node").forEach((element) => {
    element.classList.remove("fsm-active-node");
  });

  root.querySelectorAll(".fsm-active-ancestor").forEach((element) => {
    element.classList.remove("fsm-active-ancestor");
  });
}

function applyHighlights(cardState) {
  const { svg, graph, aliasToElement } = cardState;
  if (!svg) {
    return;
  }

  clearHighlightClasses(svg);

  graph.nodesById.forEach((node) => {
    const element = aliasToElement.get(node.alias);
    if (!element) {
      return;
    }

    if (node.active) {
      element.classList.add("fsm-active-node");
    } else if (node.hasActiveDescendant) {
      element.classList.add("fsm-active-ancestor");
    }
  });
}

function createDiagramCardShell(fsmName) {
  const card = document.createElement("section");
  card.className = "diagram-card";

  const header = document.createElement("div");
  header.className = "diagram-card-header";

  const title = document.createElement("h3");
  title.textContent = fsmName;

  const subtitle = document.createElement("p");

  header.appendChild(title);
  header.appendChild(subtitle);
  card.appendChild(header);

  const viewport = document.createElement("div");
  viewport.className = "diagram-viewport";

  const zoomWrapper = document.createElement("div");
  zoomWrapper.className = "diagram-zoom-wrapper";
  zoomWrapper.dataset.zoom = "1";

  const content = document.createElement("div");
  content.className = "diagram-content";

  zoomWrapper.appendChild(content);
  viewport.appendChild(zoomWrapper);
  card.appendChild(viewport);

  return {
    card,
    subtitle,
    viewport,
    zoomWrapper,
    content,
  };
}

async function buildDiagramCard(fsmName, rawStates, index, renderGeneration) {
  const graph = prepareGraph(fsmName, rawStates, index);
  const shell = createDiagramCardShell(fsmName);

  shell.subtitle.textContent = `${graph.visibleNodeCount} visible state node(s), ${graph.transitionCount} transition(s)`;

  const renderId = `yasmin_mermaid_${renderGeneration}_${index}`;
  const { svg, bindFunctions } = await mermaid.render(
    renderId,
    graph.definition,
  );

  if (renderGeneration !== state.renderGeneration) {
    return null;
  }

  const tempContainer = document.createElement("div");
  tempContainer.innerHTML = svg;

  if (bindFunctions) {
    bindFunctions(tempContainer);
  }

  removeInitialStateArtifacts(tempContainer);

  const renderedSvg = tempContainer.querySelector("svg");
  if (!renderedSvg) {
    throw new Error("Mermaid did not return an SVG");
  }

  shell.content.replaceChildren(renderedSvg);

  const aliasToElement = indexRenderedNodes(renderedSvg, graph);
  const viewportKey = getViewportKey(fsmName);
  shell.viewport.dataset.viewportKey = viewportKey;

  enableViewportInteraction(shell.viewport, viewportKey, shell.zoomWrapper);

  return {
    fsmName,
    graph,
    card: shell.card,
    subtitle: shell.subtitle,
    viewport: shell.viewport,
    zoomWrapper: shell.zoomWrapper,
    content: shell.content,
    svg: renderedSvg,
    aliasToElement,
    viewportKey,
  };
}

function attachDiagramCard(cardState) {
  diagramList.appendChild(cardState.card);
  restoreViewportPosition(
    cardState.viewport,
    cardState.viewportKey,
    cardState.zoomWrapper,
  );
  applyHighlights(cardState);
}

async function renderStructure() {
  state.renderGeneration += 1;
  const renderGeneration = state.renderGeneration;
  const selectedFsms = getSelectedFsms();

  if (selectedFsms.length === 0) {
    emptyState.classList.remove("hidden");
    viewerPanel.classList.add("hidden");
    clearElement(diagramList);
    state.diagramCache.clear();
    return;
  }

  emptyState.classList.add("hidden");
  viewerPanel.classList.remove("hidden");
  clearElement(diagramList);
  state.diagramCache.clear();

  viewerTitle.textContent =
    state.currentFsm === "ALL" ? "All FSMs" : state.currentFsm;

  const totalNodes = selectedFsms.reduce(
    (sum, item) => sum + item.states.length,
    0,
  );
  viewerSubtitle.textContent = `${selectedFsms.length} FSM(s), ${totalNodes} raw state node(s)`;

  for (let index = 0; index < selectedFsms.length; index += 1) {
    const item = selectedFsms[index];
    const cardState = await buildDiagramCard(
      item.name,
      item.states,
      index,
      renderGeneration,
    );

    if (!cardState) {
      return;
    }

    state.diagramCache.set(getCacheKey(item.name), cardState);
    attachDiagramCard(cardState);
  }
}

function updateActiveStatesOnly() {
  const selectedFsms = getSelectedFsms();

  viewerTitle.textContent =
    state.currentFsm === "ALL" ? "All FSMs" : state.currentFsm;

  const totalNodes = selectedFsms.reduce(
    (sum, item) => sum + item.states.length,
    0,
  );
  viewerSubtitle.textContent = `${selectedFsms.length} FSM(s), ${totalNodes} raw state node(s)`;

  selectedFsms.forEach((item, index) => {
    const graph = prepareGraph(item.name, item.states, index);
    const cacheKey = getCacheKey(item.name);
    const cardState = state.diagramCache.get(cacheKey);

    if (!cardState) {
      return;
    }

    cardState.graph = graph;
    cardState.subtitle.textContent = `${graph.visibleNodeCount} visible state node(s), ${graph.transitionCount} transition(s)`;
    applyHighlights(cardState);
  });
}

async function updateView(forceStructure = false) {
  const structureSignature = getStructureSignature(state.fsms);
  const activeSignature = getActiveSignature(state.fsms);

  const structureChanged =
    forceStructure || structureSignature !== state.lastStructureSignature;

  if (structureChanged) {
    state.lastStructureSignature = structureSignature;
    state.lastActiveSignature = activeSignature;
    await renderStructure();
    return;
  }

  if (activeSignature !== state.lastActiveSignature) {
    state.lastActiveSignature = activeSignature;
    updateActiveStatesOnly();
  }
}

async function pollFsms() {
  try {
    const response = await fetch("/api/fsms", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const nextFsms = await response.json();

    state.lastSuccessMs = Date.now();
    setStatus("Connected", "status-ok");

    state.fsms = nextFsms;
    updateFsmSelector();
    await updateView(false);
  } catch (error) {
    const stale = Date.now() - state.lastSuccessMs > 3000;
    setStatus(
      stale ? "Waiting for data" : "Reconnecting",
      stale ? "status-stale" : "status-connecting",
    );
    console.error(error);
  }
}

fsmSelect.addEventListener("change", async (event) => {
  state.currentFsm = event.target.value;
  await updateView(true);
});

hideNestedCheckbox.addEventListener("change", async (event) => {
  state.hideNested = event.target.checked;
  await updateView(true);
});

showActiveCheckbox.addEventListener("change", async (event) => {
  state.showOnlyActive = event.target.checked;
  await updateView(true);
});

refreshButton.addEventListener("click", async () => {
  state.lastStructureSignature = "";
  state.lastActiveSignature = "";
  await updateView(true);
});

setStatus("Connecting", "status-connecting");
await pollFsms();
window.setInterval(() => {
  pollFsms();
}, 750);
