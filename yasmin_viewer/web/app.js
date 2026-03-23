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

const state = {
  fsms: {},
  currentFsm: "ALL",
  hideNested: false,
  showOnlyActive: false,
  cardsPerRow: 2,
  lastSuccessMs: 0,
  renderGeneration: 0,
  lastStructureSignature: "",
  lastActiveSignature: "",
  diagramCache: new Map(),
  viewportState: new Map(),
};

const fsmSelect = document.getElementById("fsm-select");
const cardsPerRowSelect = document.getElementById("cards-per-row");
const hideNestedCheckbox = document.getElementById("hide-nested");
const showActiveCheckbox = document.getElementById("show-active");
const fitButton = document.getElementById("fit-button");
const centerButton = document.getElementById("center-button");
const resetZoomButton = document.getElementById("reset-zoom-button");
const emptyState = document.getElementById("empty-state");
const viewerPanel = document.getElementById("viewer-panel");
const statusBadge = document.getElementById("status-badge");
const diagramList = document.getElementById("diagram-list");

mermaid.initialize({
  startOnLoad: false,
  theme: "base",
  securityLevel: "strict",
  htmlLabels: false,
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
    cardsPerRow: state.cardsPerRow,
  });
}

function getActiveSignature(fsms) {
  return stableStringify({
    fsms,
    currentFsm: state.currentFsm,
    hideNested: state.hideNested,
    showOnlyActive: state.showOnlyActive,
    cardsPerRow: state.cardsPerRow,
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

function updateCardsPerRowSelector() {
  cardsPerRowSelect.value = String(state.cardsPerRow);
}

function updateDiagramGridColumns() {
  diagramList.style.setProperty("--fsm-columns", String(state.cardsPerRow));
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
      const outcomeLabel = Array.isArray(transition)
        ? transition[0]
        : transition.outcome;
      const targetName = Array.isArray(transition)
        ? transition[1]
        : transition.state;
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
    cardsPerRow: state.cardsPerRow,
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
    centerContent: false,
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
    centerContent: viewport.dataset.centerContent === "true",
  });
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

function getViewportInnerSize(viewport) {
  const style = window.getComputedStyle(viewport);
  const paddingLeft = Number.parseFloat(style.paddingLeft || "0");
  const paddingRight = Number.parseFloat(style.paddingRight || "0");
  const paddingTop = Number.parseFloat(style.paddingTop || "0");
  const paddingBottom = Number.parseFloat(style.paddingBottom || "0");

  return {
    width: Math.max(0, viewport.clientWidth - paddingLeft - paddingRight),
    height: Math.max(0, viewport.clientHeight - paddingTop - paddingBottom),
  };
}

function getViewportContentSize(zoomWrapper) {
  const content = zoomWrapper.firstElementChild;
  if (!content) {
    return { width: 0, height: 0 };
  }

  const svg = content.querySelector("svg");
  if (svg && typeof svg.getBBox === "function") {
    try {
      const bbox = svg.getBBox();
      return {
        width: Math.ceil(bbox.width + bbox.x + 16),
        height: Math.ceil(bbox.height + bbox.y + 16),
      };
    } catch (error) {
      console.error(error);
    }
  }

  return {
    width: Math.ceil(content.scrollWidth),
    height: Math.ceil(content.scrollHeight),
  };
}

function getViewportContentOffset(viewport, zoomWrapper) {
  const centerContent = viewport.dataset.centerContent === "true";
  if (!centerContent) {
    return { left: 0, top: 0 };
  }

  const { width, height } = getViewportContentSize(zoomWrapper);
  const { width: innerWidth, height: innerHeight } = getViewportInnerSize(viewport);
  const zoom = getZoom(zoomWrapper);

  return {
    left: Math.max(0, (innerWidth - width * zoom) / 2),
    top: Math.max(0, (innerHeight - height * zoom) / 2),
  };
}

function applyViewportContentOffset(viewport, zoomWrapper) {
  const offset = getViewportContentOffset(viewport, zoomWrapper);
  zoomWrapper.style.marginLeft = `${offset.left}px`;
  zoomWrapper.style.marginTop = `${offset.top}px`;
}

function centerViewport(viewport, zoomWrapper) {
  viewport.dataset.centerContent = "true";
  applyViewportContentOffset(viewport, zoomWrapper);

  const { width, height } = getViewportContentSize(zoomWrapper);
  const { width: innerWidth, height: innerHeight } = getViewportInnerSize(viewport);
  const zoom = getZoom(zoomWrapper);
  const offset = getViewportContentOffset(viewport, zoomWrapper);

  const scaledWidth = width * zoom;
  const scaledHeight = height * zoom;

  viewport.scrollLeft =
    scaledWidth + offset.left > innerWidth
      ? Math.max(0, (scaledWidth - innerWidth) / 2)
      : 0;
  viewport.scrollTop =
    scaledHeight + offset.top > innerHeight
      ? Math.max(0, (scaledHeight - innerHeight) / 2)
      : 0;

  const viewportKey = viewport.dataset.viewportKey;
  if (viewportKey) {
    saveViewportPosition(viewport, viewportKey, zoomWrapper);
  }
}

function resetViewportZoom(viewport, zoomWrapper) {
  setZoom(zoomWrapper, 1);
  centerViewport(viewport, zoomWrapper);
}

function fitViewport(viewport, zoomWrapper) {
  const { width, height } = getViewportContentSize(zoomWrapper);
  const { width: innerWidth, height: innerHeight } = getViewportInnerSize(viewport);

  if (width <= 0 || height <= 0 || innerWidth <= 0 || innerHeight <= 0) {
    return;
  }

  const fitMargin = 0.96;
  let fitZoom = clampZoom(
    Math.min(innerWidth / width, innerHeight / height) * fitMargin,
  );

  setZoom(zoomWrapper, fitZoom);
  viewport.dataset.centerContent = "true";
  applyViewportContentOffset(viewport, zoomWrapper);
  centerViewport(viewport, zoomWrapper);

  for (let iteration = 0; iteration < 6; iteration += 1) {
    const horizontalOverflow = viewport.scrollWidth - viewport.clientWidth;
    const verticalOverflow = viewport.scrollHeight - viewport.clientHeight;

    if (horizontalOverflow <= 1 && verticalOverflow <= 1) {
      break;
    }

    fitZoom = clampZoom(fitZoom * 0.98);
    setZoom(zoomWrapper, fitZoom);
    applyViewportContentOffset(viewport, zoomWrapper);
    centerViewport(viewport, zoomWrapper);
  }

  const viewportKey = viewport.dataset.viewportKey;
  if (viewportKey) {
    saveViewportPosition(viewport, viewportKey, zoomWrapper);
  }
}

function restoreViewportPosition(viewport, viewportKey, zoomWrapper) {
  const saved = getViewportState(viewportKey);
  setZoom(zoomWrapper, saved.zoom);
  viewport.dataset.centerContent = saved.centerContent ? "true" : "false";
  applyViewportContentOffset(viewport, zoomWrapper);
  viewport.scrollLeft = saved.scrollLeft;
  viewport.scrollTop = saved.scrollTop;
}

function zoomAroundPoint(viewport, zoomWrapper, nextZoom, clientX, clientY) {
  const previousZoom = getZoom(zoomWrapper);
  const clampedZoom = clampZoom(nextZoom);

  if (Math.abs(clampedZoom - previousZoom) < 1e-6) {
    return;
  }

  const rect = viewport.getBoundingClientRect();
  const previousOffset = getViewportContentOffset(viewport, zoomWrapper);

  const offsetX =
    clientX - rect.left + viewport.scrollLeft - previousOffset.left;
  const offsetY =
    clientY - rect.top + viewport.scrollTop - previousOffset.top;

  const contentX = offsetX / previousZoom;
  const contentY = offsetY / previousZoom;

  viewport.dataset.centerContent = "false";
  setZoom(zoomWrapper, clampedZoom);
  applyViewportContentOffset(viewport, zoomWrapper);

  const nextOffset = getViewportContentOffset(viewport, zoomWrapper);

  viewport.scrollLeft =
    contentX * clampedZoom + nextOffset.left - (clientX - rect.left);
  viewport.scrollTop =
    contentY * clampedZoom + nextOffset.top - (clientY - rect.top);

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

    viewport.dataset.centerContent = "false";
    applyViewportContentOffset(viewport, zoomWrapper);

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

  viewport.addEventListener("pointerup", stopDragging);
  viewport.addEventListener("pointercancel", stopDragging);
  viewport.addEventListener("lostpointercapture", stopDragging);

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
      const insideViewport =
        event.target instanceof Node && viewport.contains(event.target);
      if (!insideViewport) {
        return;
      }

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

  header.appendChild(title);
  card.appendChild(header);

  const viewport = document.createElement("div");
  viewport.className = "diagram-viewport";

  const zoomWrapper = document.createElement("div");
  zoomWrapper.className = "diagram-zoom-wrapper";
  zoomWrapper.dataset.zoom = "1";
  viewport.dataset.centerContent = "false";

  const content = document.createElement("div");
  content.className = "diagram-content";

  zoomWrapper.appendChild(content);
  viewport.appendChild(zoomWrapper);
  card.appendChild(viewport);

  return {
    card,
    viewport,
    zoomWrapper,
    content,
  };
}

async function buildDiagramCard(fsmName, rawStates, index, renderGeneration) {
  const graph = prepareGraph(fsmName, rawStates, index);
  const shell = createDiagramCardShell(fsmName);

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

  if (!state.viewportState.has(cardState.viewportKey)) {
    requestAnimationFrame(() => {
      fitViewport(cardState.viewport, cardState.zoomWrapper);
    });
  }
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
  updateDiagramGridColumns();

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

  selectedFsms.forEach((item, index) => {
    const graph = prepareGraph(item.name, item.states, index);
    const cacheKey = getCacheKey(item.name);
    const cardState = state.diagramCache.get(cacheKey);

    if (!cardState) {
      return;
    }

    cardState.graph = graph;
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
    updateCardsPerRowSelector();
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

function applyToVisibleCards(callback) {
  state.diagramCache.forEach((cardState) => {
    if (!cardState.card.isConnected) {
      return;
    }

    callback(cardState);
  });
}

fsmSelect.addEventListener("change", async (event) => {
  state.currentFsm = event.target.value;
  await updateView(true);
});

cardsPerRowSelect.addEventListener("change", async (event) => {
  state.cardsPerRow = Number(event.target.value);
  updateDiagramGridColumns();
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

fitButton.addEventListener("click", () => {
  applyToVisibleCards((cardState) => {
    fitViewport(cardState.viewport, cardState.zoomWrapper);
  });
});

centerButton.addEventListener("click", () => {
  applyToVisibleCards((cardState) => {
    centerViewport(cardState.viewport, cardState.zoomWrapper);
  });
});

resetZoomButton.addEventListener("click", () => {
  applyToVisibleCards((cardState) => {
    resetViewportZoom(cardState.viewport, cardState.zoomWrapper);
  });
});

updateCardsPerRowSelector();
updateDiagramGridColumns();
setStatus("Connecting", "status-connecting");
await pollFsms();
window.setInterval(() => {
  pollFsms();
}, 750);
