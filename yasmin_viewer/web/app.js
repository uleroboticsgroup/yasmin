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
  lastSuccessMs: 0,
  lastDataSignature: "",
};

const renderedGraphs = new Map();

const fsmSelect = document.getElementById("fsm-select");
const hideNestedCheckbox = document.getElementById("hide-nested");
const showActiveCheckbox = document.getElementById("show-active");
const refreshButton = document.getElementById("refresh-button");
const emptyState = document.getElementById("empty-state");
const viewerPanel = document.getElementById("viewer-panel");
const viewerTitle = document.getElementById("viewer-title");
const viewerSubtitle = document.getElementById("viewer-subtitle");
const statusBadge = document.getElementById("status-badge");
const graphList = document.getElementById("graph-list");

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

function getDataSignature(fsms) {
  return stableStringify(fsms);
}

function clearElement(element) {
  while (element.firstChild) {
    element.removeChild(element.firstChild);
  }
}

function setStatus(text, cssClass) {
  statusBadge.textContent = text;
  statusBadge.className = `status-badge ${cssClass}`;
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
  return Object.keys(state.fsms)
    .sort()
    .filter((name) => state.currentFsm === "ALL" || state.currentFsm === name)
    .map((name) => ({
      name,
      states: state.fsms[name],
    }))
    .filter(({ states }) => Array.isArray(states) && states.length > 0);
}

function cloneNodes(rawStates, fsmName) {
  const nodesById = new Map();

  rawStates.forEach((entry) => {
    nodesById.set(entry.id, {
      ...entry,
      graphId: `${fsmName}::${entry.id}`,
      active: false,
      activeAncestor: false,
      visible: true,
      syntheticOutcome: false,
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
      parent.activeAncestor = true;
      parentId = parent.parent;
    }
  });
}

function computeVisibility(nodesById) {
  if (state.showOnlyActive) {
    nodesById.forEach((node) => {
      node.visible = node.parent === -1 || node.active || node.activeAncestor;
    });
  } else {
    nodesById.forEach((node) => {
      node.visible = true;
    });
  }

  if (state.hideNested) {
    nodesById.forEach((node) => {
      if (!node.visible) {
        return;
      }

      let parentId = node.parent;
      while (nodesById.has(parentId)) {
        const parent = nodesById.get(parentId);
        if (parent.parent !== -1 && parent.is_fsm) {
          node.visible = false;
          break;
        }
        parentId = parent.parent;
      }
    });
  }
}

function buildSiblingNameMap(nodesById) {
  const siblingNameMap = new Map();

  nodesById.forEach((node) => {
    if (!node.visible) {
      return;
    }

    const key = `${node.parent}::${node.name}`;
    siblingNameMap.set(key, node);
  });

  return siblingNameMap;
}

function shouldUseCompoundParent(node, nodesById) {
  if (node.parent === -1) {
    return false;
  }

  if (!nodesById.has(node.parent)) {
    return false;
  }

  const parent = nodesById.get(node.parent);
  return parent.visible && parent.is_fsm && !state.hideNested;
}

function getNodeClasses(node) {
  const classes = [];

  if (node.syntheticOutcome) {
    classes.push("outcome");
  } else if (node.is_fsm) {
    classes.push("fsm");
  } else {
    classes.push("state");
  }

  if (node.current_state === -2) {
    classes.push("concurrence");
  }

  if (node.active) {
    classes.push("active");
  } else if (node.activeAncestor) {
    classes.push("active-ancestor");
  }

  return classes.join(" ");
}

function buildGraphModel(fsmName, rawStates) {
  const nodesById = cloneNodes(rawStates, fsmName);
  computeActiveFlags(nodesById);
  computeVisibility(nodesById);

  const siblingNameMap = buildSiblingNameMap(nodesById);
  const syntheticOutcomeNodes = new Map();

  function getOrCreateOutcomeNode(sourceNode, outcomeLabel, targetName) {
    const outcomeId = `${sourceNode.graphId}::outcome::${outcomeLabel}::${targetName}`;
    if (syntheticOutcomeNodes.has(outcomeId)) {
      return syntheticOutcomeNodes.get(outcomeId);
    }

    const node = {
      graphId: outcomeId,
      label: targetName,
      parent: shouldUseCompoundParent(sourceNode, nodesById)
        ? nodesById.get(sourceNode.parent).graphId
        : undefined,
      classes: "outcome",
      syntheticOutcome: true,
    };

    syntheticOutcomeNodes.set(outcomeId, node);
    return node;
  }

  const elements = [];
  const activeNodeIds = [];

  nodesById.forEach((node) => {
    if (!node.visible) {
      return;
    }

    const element = {
      group: "nodes",
      data: {
        id: node.graphId,
        label: node.name,
        parent: shouldUseCompoundParent(node, nodesById)
          ? nodesById.get(node.parent).graphId
          : undefined,
        outcomes: Array.isArray(node.outcomes) ? node.outcomes.join(" | ") : "",
        subtitle:
          node.current_state === -2
            ? "Concurrence"
            : node.is_fsm
              ? "FSM"
              : "State",
      },
      classes: getNodeClasses(node),
    };

    elements.push(element);

    if (node.active) {
      activeNodeIds.push(node.graphId);
    }
  });

  const edgeKeys = new Set();

  nodesById.forEach((node) => {
    if (!node.visible) {
      return;
    }

    (node.transitions || []).forEach((transition, index) => {
      const outcomeLabel = transition[0];
      const targetName = transition[1];
      const siblingKey = `${node.parent}::${targetName}`;

      let targetId;
      if (siblingNameMap.has(siblingKey)) {
        targetId = siblingNameMap.get(siblingKey).graphId;
      } else {
        const syntheticNode = getOrCreateOutcomeNode(
          node,
          outcomeLabel,
          targetName,
        );
        targetId = syntheticNode.graphId;
      }

      const edgeId = `${node.graphId}::${index}::${outcomeLabel}::${targetId}`;
      if (edgeKeys.has(edgeId)) {
        return;
      }

      edgeKeys.add(edgeId);
      elements.push({
        group: "edges",
        data: {
          id: edgeId,
          source: node.graphId,
          target: targetId,
          label: outcomeLabel,
        },
        classes: node.active ? "active-edge" : "",
      });
    });
  });

  syntheticOutcomeNodes.forEach((node) => {
    elements.push({
      group: "nodes",
      data: {
        id: node.graphId,
        label: node.label,
        parent: node.parent,
        outcomes: "",
        subtitle: "Outcome",
      },
      classes: node.classes,
    });
  });

  const structureSignature = stableStringify(
    elements.map((element) => ({
      group: element.group,
      data: element.data,
      classes: element.classes
        .split(" ")
        .filter(
          (item) =>
            item !== "active" &&
            item !== "active-ancestor" &&
            item !== "active-edge",
        )
        .sort()
        .join(" "),
    })),
  );

  const activeSignature = stableStringify({
    activeNodeIds: elements
      .filter(
        (element) =>
          element.group === "nodes" && element.classes.includes("active"),
      )
      .map((element) => element.data.id)
      .sort(),
    activeAncestorIds: elements
      .filter(
        (element) =>
          element.group === "nodes" &&
          element.classes.includes("active-ancestor"),
      )
      .map((element) => element.data.id)
      .sort(),
    activeEdgeIds: elements
      .filter(
        (element) =>
          element.group === "edges" && element.classes.includes("active-edge"),
      )
      .map((element) => element.data.id)
      .sort(),
  });

  return {
    elements,
    structureSignature,
    activeSignature,
    totalVisibleNodes: elements.filter((element) => element.group === "nodes")
      .length,
    totalVisibleEdges: elements.filter((element) => element.group === "edges")
      .length,
  };
}

function makeStylesheet() {
  return [
    {
      selector: "node",
      style: {
        label: "data(label)",
        "text-wrap": "wrap",
        "text-max-width": 180,
        "text-valign": "center",
        "text-halign": "center",
        "font-size": 14,
        "font-weight": 700,
        color: "#0f172a",
        "background-color": "#eff6ff",
        "border-color": "#334155",
        "border-width": 2,
        width: 150,
        height: 64,
        shape: "round-rectangle",
        "overlay-opacity": 0,
      },
    },
    {
      selector: "node[state]",
      style: {
        "background-color": "#ffffff",
      },
    },
    {
      selector: "node[fsm]",
      style: {
        "background-color": "#e3f2fd",
      },
    },
    {
      selector: "node.outcome",
      style: {
        "background-color": "#fff7ed",
        "border-color": "#c2410c",
        shape: "round-rectangle",
        width: 120,
        height: 52,
      },
    },
    {
      selector: "node.concurrence",
      style: {
        shape: "diamond",
        width: 120,
        height: 120,
        "background-color": "#fff8dc",
      },
    },
    {
      selector: "node.active",
      style: {
        "background-color": "#dcedc8",
        "border-color": "#2e7d32",
        "border-width": 4,
      },
    },
    {
      selector: "node.active-ancestor",
      style: {
        "border-color": "#1e88e5",
        "border-width": 3,
      },
    },
    {
      selector: "$node > node",
      style: {
        "background-opacity": 0.08,
        "border-style": "dashed",
        "border-width": 2,
        "border-color": "#94a3b8",
        "text-valign": "top",
        "text-halign": "center",
        "padding-top": 28,
        "padding-left": 16,
        "padding-right": 16,
        "padding-bottom": 16,
        "font-size": 15,
      },
    },
    {
      selector: "edge",
      style: {
        width: 2.5,
        "curve-style": "bezier",
        "target-arrow-shape": "triangle",
        "target-arrow-color": "#64748b",
        "line-color": "#64748b",
        label: "data(label)",
        "font-size": 12,
        color: "#334155",
        "text-background-color": "#ffffff",
        "text-background-opacity": 1,
        "text-background-padding": 2,
        "text-rotation": "autorotate",
        "overlay-opacity": 0,
      },
    },
    {
      selector: "edge.active-edge",
      style: {
        width: 3.5,
        "line-color": "#1e88e5",
        "target-arrow-color": "#1e88e5",
      },
    },
  ];
}

function createGraphCard(fsmName) {
  const card = document.createElement("section");
  card.className = "graph-card";
  card.dataset.fsmName = fsmName;

  const header = document.createElement("div");
  header.className = "graph-card-header";

  const title = document.createElement("h3");
  title.textContent = fsmName;

  const subtitle = document.createElement("p");

  header.appendChild(title);
  header.appendChild(subtitle);
  card.appendChild(header);

  const scroll = document.createElement("div");
  scroll.className = "graph-scroll";

  const container = document.createElement("div");
  container.className = "graph-container";

  scroll.appendChild(container);
  card.appendChild(scroll);

  graphList.appendChild(card);

  return {
    card,
    subtitle,
    container,
  };
}

function destroyRemovedGraphs(nextNames) {
  Array.from(renderedGraphs.keys()).forEach((fsmName) => {
    if (nextNames.has(fsmName)) {
      return;
    }

    const entry = renderedGraphs.get(fsmName);
    if (entry.cy) {
      entry.cy.destroy();
    }
    if (entry.card && entry.card.parentNode) {
      entry.card.parentNode.removeChild(entry.card);
    }
    renderedGraphs.delete(fsmName);
  });
}

function updateGraphOrder(selectedFsms) {
  selectedFsms.forEach(({ name }) => {
    const entry = renderedGraphs.get(name);
    if (entry && entry.card) {
      graphList.appendChild(entry.card);
    }
  });
}

function rebuildGraph(fsmName, graphModel) {
  let entry = renderedGraphs.get(fsmName);
  if (!entry) {
    entry = createGraphCard(fsmName);
    renderedGraphs.set(fsmName, entry);
  }

  if (entry.cy) {
    entry.cy.destroy();
  }

  entry.subtitle.textContent = `${graphModel.totalVisibleNodes} visible node(s), ${graphModel.totalVisibleEdges} visible edge(s)`;

  entry.cy = cytoscape({
    container: entry.container,
    elements: graphModel.elements,
    style: makeStylesheet(),
    layout: {
      name: "breadthfirst",
      directed: true,
      padding: 30,
      spacingFactor: 1.15,
      fit: true,
      avoidOverlap: true,
      nodeDimensionsIncludeLabels: true,
    },
    wheelSensitivity: 0.2,
    minZoom: 0.2,
    maxZoom: 3.0,
  });

  entry.structureSignature = graphModel.structureSignature;
  entry.activeSignature = graphModel.activeSignature;
}

function updateActiveClasses(fsmName, graphModel) {
  const entry = renderedGraphs.get(fsmName);
  if (!entry || !entry.cy) {
    return;
  }

  entry.subtitle.textContent = `${graphModel.totalVisibleNodes} visible node(s), ${graphModel.totalVisibleEdges} visible edge(s)`;

  entry.cy.batch(() => {
    entry.cy.nodes().removeClass("active");
    entry.cy.nodes().removeClass("active-ancestor");
    entry.cy.edges().removeClass("active-edge");

    graphModel.elements.forEach((element) => {
      if (element.group === "nodes") {
        const node = entry.cy.getElementById(element.data.id);
        if (!node || node.empty()) {
          return;
        }

        if (element.classes.includes("active")) {
          node.addClass("active");
        } else if (element.classes.includes("active-ancestor")) {
          node.addClass("active-ancestor");
        }
      } else if (element.group === "edges") {
        const edge = entry.cy.getElementById(element.data.id);
        if (!edge || edge.empty()) {
          return;
        }

        if (element.classes.includes("active-edge")) {
          edge.addClass("active-edge");
        }
      }
    });
  });

  entry.activeSignature = graphModel.activeSignature;
}

function render() {
  const selectedFsms = getSelectedFsms();
  const nextNames = new Set(selectedFsms.map((item) => item.name));

  if (selectedFsms.length === 0) {
    emptyState.classList.remove("hidden");
    viewerPanel.classList.add("hidden");
    destroyRemovedGraphs(nextNames);
    return;
  }

  emptyState.classList.add("hidden");
  viewerPanel.classList.remove("hidden");

  viewerTitle.textContent =
    state.currentFsm === "ALL" ? "All FSMs" : state.currentFsm;

  const totalRawNodes = selectedFsms.reduce(
    (sum, item) => sum + item.states.length,
    0,
  );
  viewerSubtitle.textContent = `${selectedFsms.length} FSM(s), ${totalRawNodes} raw state node(s)`;

  destroyRemovedGraphs(nextNames);

  selectedFsms.forEach(({ name, states }) => {
    const graphModel = buildGraphModel(name, states);
    const current = renderedGraphs.get(name);

    if (
      !current ||
      current.structureSignature !== graphModel.structureSignature
    ) {
      rebuildGraph(name, graphModel);
      return;
    }

    if (current.activeSignature !== graphModel.activeSignature) {
      updateActiveClasses(name, graphModel);
    }
  });

  updateGraphOrder(selectedFsms);
}

async function pollFsms() {
  try {
    const response = await fetch("/api/fsms", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const nextFsms = await response.json();
    const nextSignature = getDataSignature(nextFsms);

    state.lastSuccessMs = Date.now();
    setStatus("Connected", "status-ok");

    if (nextSignature === state.lastDataSignature) {
      return;
    }

    state.fsms = nextFsms;
    state.lastDataSignature = nextSignature;
    updateFsmSelector();
    render();
  } catch (error) {
    const stale = Date.now() - state.lastSuccessMs > 3000;
    setStatus(
      stale ? "Waiting for data" : "Reconnecting",
      stale ? "status-stale" : "status-connecting",
    );
    console.error(error);
  }
}

fsmSelect.addEventListener("change", (event) => {
  state.currentFsm = event.target.value;
  render();
});

hideNestedCheckbox.addEventListener("change", (event) => {
  state.hideNested = event.target.checked;
  render();
});

showActiveCheckbox.addEventListener("change", (event) => {
  state.showOnlyActive = event.target.checked;
  render();
});

refreshButton.addEventListener("click", () => {
  render();
});

setStatus("Connecting", "status-connecting");
pollFsms();
window.setInterval(() => {
  pollFsms();
}, 500);
