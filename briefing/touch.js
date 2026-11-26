"use strict";

{
  const cases = {
    offset: {preview: 0, events: ["Jaws closed", "Start lift", "After lift"],
      caption: "Only the left pad registered contact during closure."},
    slip: {preview: 0, events: ["Jaws closed", "Start lift", "After slip"],
      caption: "Both pads registered contact at closure. Their force dropped to zero after the part slipped."},
    recovered: {preview: 2, events: ["Reclose", "Lift clear", "Stable hold"],
      caption: "The jaws reopened before descending. After the regrasp, both pads stayed loaded through the lift."}
  };
  const video = document.querySelector("#touch-video");
  const play = document.querySelector("#touch-play");
  const seek = document.querySelector("#touch-seek");
  const phase = document.querySelector("#touch-phase");
  const time = document.querySelector("#touch-time");
  const caption = document.querySelector("#touch-caption");
  const lift = document.querySelector("#touch-lift");
  const gap = document.querySelector("#touch-gap");
  const download = document.querySelector("#touch-download");
  const cards = [...document.querySelectorAll("[data-touch-case]")];
  const jumps = [...document.querySelectorAll(".touch-jumps button")];
  const observations = [...document.querySelectorAll("[data-feed]")];
  const padRow = document.querySelector("#touch-pads");
  const pointOutput = document.querySelector("#touch-point");
  const svgNamespace = "http://www.w3.org/2000/svg";
  const sides = ["Left pad", "Right pad"];
  const readings = new Map();
  let selected;
  let activeRecording;
  let currentFrame;
  let padViews = [];
  let focusedPoint;
  let pendingTime;
  let frameCallback;

  function svg(parent, element, attributes) {
    const node = document.createElementNS(svgNamespace, element);
    for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value);
    parent.append(node);
    return node;
  }

  function recording(id) {
    if (!readings.has(id)) {
      readings.set(id, fetch(`data/touch-${id}.json?v=hd`).then(response => {
        if (!response.ok) throw new Error("Recording unavailable");
        return response.json();
      }).catch(error => {
        readings.delete(id);
        throw error;
      }));
    }
    return readings.get(id);
  }

  function nearest(frames, second) {
    return frames.reduce((best, frame) =>
      Math.abs(frame.s - second) < Math.abs(best.s - second) ? frame : best);
  }

  function measure(value, digits = 2) {
    return (Math.abs(value) < 0.5 * 10 ** -digits ? 0 : value).toFixed(digits);
  }

  function inspectPoint(side, index, node) {
    focusedPoint?.node.removeAttribute("data-selected");
    focusedPoint = {side, index, node};
    node.setAttribute("data-selected", "true");
    showPoint();
  }

  function showPoint() {
    if (!focusedPoint || !currentFrame) return;
    const {side, index} = focusedPoint;
    const [, fx, fy, fz] = currentFrame.active[side].find(entry => entry[0] === index) || [index, 0, 0, 0];
    pointOutput.hidden = false;
    pointOutput.textContent = `${sides[side]} · Point ${index + 1} · Fx ${measure(fx, 3)} · Fy ${measure(fy, 3)} · Fz ${measure(fz, 3)} N`;
  }

  function makeMap(container, coords, side, interactive) {
    const map = svg(container, "svg", {
      viewBox: interactive ? "-19 -34 38 41" : "-16 -32 32 37",
      class: "pad-map", "aria-label": `${sides[side]} force map`, ...(interactive ? {} : {"aria-hidden": "true"})
    });
    svg(map, "rect", {x: -15.5, y: -31.25, width: 31, height: 34.6, rx: 3, class: "pad-frame"});
    if (interactive) {
      svg(map, "text", {x: -14.5, y: -28.8, class: "pad-axis"}).textContent = "↑ +Y";
      svg(map, "text", {x: 9.3, y: 5.6, class: "pad-axis"}).textContent = "→ +X";
    }
    const dots = coords.map(([x, y], index) => {
      const group = svg(map, "g", interactive ? {
        class: "taxel", role: "button", tabindex: index === 0 ? 0 : -1,
        "aria-label": `${sides[side]}, point ${index + 1}`
      } : {});
      const dot = svg(group, "circle", {cx: x, cy: -y, r: .54, class: "pad-dot"});
      if (interactive) {
        svg(group, "circle", {cx: x, cy: -y, r: 1.6, fill: "transparent"});
        group.addEventListener("click", () => inspectPoint(side, index, group));
        group.addEventListener("keydown", event => {
          if (event.key === "Enter" || event.key === " ") {
            event.preventDefault();
            inspectPoint(side, index, group);
          }
          const keys = {ArrowRight: 1, ArrowDown: 1, ArrowLeft: -1, ArrowUp: -1};
          if (keys[event.key]) {
            event.preventDefault();
            event.stopPropagation();
            const next = dots[(index + keys[event.key] + dots.length) % dots.length].group;
            group.setAttribute("tabindex", -1);
            next.setAttribute("tabindex", 0);
            next.focus();
          }
        });
      }
      return {group, dot, x, y: -y};
    });
    const vectors = svg(map, "g", {"aria-hidden": "true", class: "pad-vectors"});
    const cop = svg(map, "g", {class: "pad-cop", visibility: "hidden", "aria-hidden": "true"});
    svg(cop, "circle", {r: 1});
    svg(cop, "path", {d: "M -1.5 0 H 1.5 M 0 -1.5 V 1.5"});
    return {dots, vectors, cop, lastActive: []};
  }

  function paintMap(map, forces) {
    for (const index of map.lastActive) {
      const {dot} = map.dots[index];
      dot.setAttribute("r", .54);
      dot.style.removeProperty("fill");
    }
    map.lastActive = forces.map(([index]) => index);
    map.vectors.replaceChildren();
    let weight = 0;
    let centerX = 0;
    let centerY = 0;
    for (const [index, fx, fy, fz] of forces) {
      const {dot, x, y} = map.dots[index];
      if (fz > 0) {
        weight += fz;
        centerX += x * fz;
        centerY += y * fz;
      }
      if (fz > .004) {
        const intensity = Math.sqrt(Math.min(1, fz / 1.8));
        dot.setAttribute("r", .54 + 1.14 * intensity);
        dot.style.fill = `hsl(${276 - 240 * intensity} 95% ${49 + 29 * intensity}%)`;
      }
      const shear = Math.hypot(fx, fy);
      if (shear <= .007) continue;
      const dx = fx / shear, dy = -fy / shear;
      const length = Math.min(7, 1.4 + 23 * shear);
      const ex = x + dx * length, ey = y + dy * length;
      const bx = ex - .92 * dx, by = ey - .92 * dy;
      svg(map.vectors, "line", {x1: x, y1: y, x2: bx, y2: by, class: "pad-arrow"});
      svg(map.vectors, "polygon", {points: `${ex},${ey} ${bx - .43 * dy},${by + .43 * dx} ${bx + .43 * dy},${by - .43 * dx}`});
    }
    map.cop.setAttribute("visibility", weight >= .15 ? "visible" : "hidden");
    if (weight >= .15) map.cop.setAttribute("transform", `translate(${centerX / weight} ${centerY / weight})`);
  }

  function makePad(side, coords) {
    const card = document.createElement("div");
    card.className = "replay-pad";
    card.innerHTML = `<div class="pad-head"><h4>${sides[side]}</h4><output class="pad-total">0.00 N</output></div>
      <div class="pad-values">${["Fx", "Fy", "Fz"].map(axis => `<div><span>${axis} (N)</span><output>0.00</output></div>`).join("")}</div>
      <p class="pad-active">0 of 77 active</p>`;
    const mapContainer = document.createElement("div");
    card.querySelector(".pad-head").after(mapContainer);
    const map = makeMap(mapContainer, coords, side, true);
    padRow.append(card);
    return {map, total: card.querySelector(".pad-total"),
      axes: [...card.querySelectorAll(".pad-values output")], active: card.querySelector(".pad-active")};
  }

  function render(second, force = false) {
    if (!activeRecording) return;
    const frame = nearest(activeRecording.frames, second);
    if (!force && frame === currentFrame) return;
    currentFrame = frame;
    phase.textContent = frame.phase;
    time.textContent = `${frame.s.toFixed(2)} s`;
    seek.value = (frame.s - activeRecording.videoStart).toFixed(2);
    seek.setAttribute("aria-valuetext", `${frame.s.toFixed(2)} seconds in simulation`);
    lift.textContent = `${measure(frame.lift, 1)} mm`;
    gap.textContent = `${measure(frame.gap, 1)} mm`;
    padViews.forEach((pad, side) => {
      const {active, totals} = frame;
      pad.total.textContent = `${measure(Math.max(0, totals[side][2]))} N`;
      pad.axes.forEach((axis, index) => axis.textContent = measure(totals[side][index]));
      pad.active.textContent = `${active[side].filter(entry => entry[3] >= .02).length} of 77 active`;
      paintMap(pad.map, active[side]);
    });
    const event = activeRecording.jumps.findIndex(value => Math.abs(frame.s - value) < .02);
    jumps.forEach((jump, index) => jump.setAttribute("aria-pressed", String(video.paused && index === event)));
    showPoint();
  }

  function jumpTo(second) {
    if (!activeRecording) return;
    video.pause();
    const start = activeRecording.videoStart;
    const bounded = Math.max(start, Math.min(second, activeRecording.frames.at(-1).s));
    pendingTime = bounded - start;
    if (video.readyState >= HTMLMediaElement.HAVE_METADATA) {
      video.currentTime = pendingTime;
      pendingTime = undefined;
    }
    render(bounded, true);
  }

  function highlightCard(card, data) {
    const id = card.dataset.touchCase;
    const frame = nearest(data.frames, data.jumps[cases[id].preview]);
    const mini = document.createElement("span");
    mini.className = "case-forces";
    const maps = document.createElement("span");
    maps.className = "mini-maps";
    for (const side of [0,1]) paintMap(makeMap(maps, data.coords, side, false), frame.active[side]);
    const readout = document.createElement("small");
    readout.textContent = `Fz · L ${measure(frame.totals[0][2])} / R ${measure(frame.totals[1][2])} N`;
    mini.append(maps, readout);
    card.append(mini);
  }

  async function selectCase(id) {
    selected = id;
    video.pause();
    pendingTime = undefined;
    activeRecording = undefined;
    currentFrame = undefined;
    focusedPoint = undefined;
    pointOutput.hidden = true;
    play.disabled = seek.disabled = true;
    for (const jump of jumps) jump.disabled = true;
    phase.textContent = "Loading recording";
    caption.textContent = cases[id].caption;
    for (const card of cards) card.setAttribute("aria-pressed", String(card.dataset.touchCase === id));

    try {
      const data = await recording(id);
      if (selected !== id) return;
      activeRecording = data;
      padRow.replaceChildren();
      padViews = sides.map((_, i) => makePad(i, data.coords));
      seek.max = (data.frames.at(-1).s - data.videoStart).toFixed(2);
      for (const [i, jump] of jumps.entries()) {
        jump.textContent = cases[id].events[i];
        jump.disabled = false;
      }
      play.disabled = seek.disabled = false;
      for (const link of observations) {
        const url = `media/observations/${id}-${link.dataset.feed}.png`;
        link.href = url;
        link.querySelector("img").src = url;
      }
      download.href = `data/touch-${id}.json?v=hd`;
      download.download = `touch-${id}.json`;
      const url = `media/touch-${id}.mp4?v=hd`;
      if (video.getAttribute("src") !== url) {
        video.poster = `media/touch-${id}.jpg?v=hd`;
        video.src = url;
      }
      jumpTo(data.jumps[cases[id].preview]);
    } catch {
      if (selected === id) phase.textContent = "Recording unavailable. Select the trial to retry.";
    }
  }

  for (const card of cards) {
    const id = card.dataset.touchCase;
    recording(id).then(data => highlightCard(card, data)).catch(() => {});
    card.addEventListener("click", () => {
      if (id !== selected || !activeRecording) selectCase(id);
    });
  }
  for (const [index, button] of jumps.entries()) {
    button.addEventListener("click", () => jumpTo(activeRecording.jumps[index]));
  }
  seek.addEventListener("input", () => jumpTo(activeRecording.videoStart + Number(seek.value)));
  play.addEventListener("click", () => {
    if (!video.paused) {
      video.pause();
      return;
    }
    if (video.currentTime >= video.duration - .05) video.currentTime = 0;
    video.play().catch(() => { phase.textContent = "Replay unavailable. Select the trial to retry."; });
  });
  video.addEventListener("loadedmetadata", () => {
    if (pendingTime === undefined) return;
    video.currentTime = pendingTime;
    pendingTime = undefined;
  });
  function updateFrame(_, metadata) {
    if (activeRecording) render(activeRecording.videoStart + metadata.mediaTime);
    frameCallback = video.requestVideoFrameCallback(updateFrame);
  }
  video.addEventListener("play", () => {
    play.textContent = "Pause";
    for (const jump of jumps) jump.setAttribute("aria-pressed", "false");
    if (video.requestVideoFrameCallback && frameCallback === undefined) frameCallback = video.requestVideoFrameCallback(updateFrame);
  });
  video.addEventListener("pause", () => {
    play.textContent = "Play";
    if (frameCallback !== undefined) video.cancelVideoFrameCallback(frameCallback);
    frameCallback = undefined;
  });
  function updateFromVideo() {
    if (activeRecording && pendingTime === undefined && !video.seeking) {
      render(activeRecording.videoStart + video.currentTime);
    }
  }
  video.addEventListener("seeked", updateFromVideo);
  if (!video.requestVideoFrameCallback) video.addEventListener("timeupdate", updateFromVideo);
  new IntersectionObserver(([entry]) => {
    if (!entry.isIntersecting) video.pause();
  }).observe(document.querySelector("#touch"));
  selectCase("offset");
}
