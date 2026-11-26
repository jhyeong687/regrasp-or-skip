"use strict";

{
  const cases = {
    offset: "Only the left pad registered contact during closure.",
    slip: "Both pads registered contact at closure. Their force dropped to zero after the part slipped.",
    recovered: "Both pads remained loaded through the lift after the jaws closed again."
  };
  const video = document.querySelector("#touch-video");
  const play = document.querySelector("#touch-play");
  const restart = document.querySelector("#touch-restart");
  const caseSelect = document.querySelector("#touch-case");
  const detailTitle = document.querySelector("#touch-detail-title");
  const seek = document.querySelector("#touch-seek");
  const phase = document.querySelector("#touch-phase");
  const time = document.querySelector("#touch-time");
  const caption = document.querySelector("#touch-caption");
  const lift = document.querySelector("#touch-lift");
  const gap = document.querySelector("#touch-gap");
  const download = document.querySelector("#touch-download");
  const comparisons = [...document.querySelectorAll("[data-comparison]")];
  const jumps = [...document.querySelectorAll(".touch-jumps button")];
  const observations = [...document.querySelectorAll("[data-feed]")];
  const padRow = document.querySelector("#touch-pads");
  const pointOutput = document.querySelector("#touch-point");
  const mapMode = document.querySelector("#touch-mode");
  const svgNamespace = "http://www.w3.org/2000/svg";
  const sides = ["Left pad", "Right pad"];
  const readings = new Map();
  let selectionVersion = 0;
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
      readings.set(id, fetch(`data/touch-${id}.json?v=compare`).then(response => {
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
    const [, fx, fy, fz] = currentFrame[mapMode.value][side].find(entry => entry[0] === index) || [index, 0, 0, 0];
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
    const max = Number(seek.max);
    const elapsed = Math.max(0, Math.min(second - activeRecording.videoStart, max));
    time.textContent = `${elapsed.toFixed(2)} / ${max.toFixed(2)} s`;
    seek.value = elapsed.toFixed(2);
    seek.setAttribute("aria-valuetext", `${elapsed.toFixed(2)} of ${max.toFixed(2)} seconds in this clip`);
    if (!force && frame === currentFrame) return;
    currentFrame = frame;
    phase.textContent = `${frame.phase} · Sim. ${frame.s.toFixed(2)} s`;
    lift.textContent = `${measure(frame.lift, 1)} mm`;
    gap.textContent = `${measure(frame.gap, 1)} mm`;
    padViews.forEach((pad, side) => {
      const {totals} = frame;
      const displayed = frame[mapMode.value][side];
      pad.total.textContent = `${measure(Math.max(0, totals[side][2]))} N`;
      pad.axes.forEach((axis, index) => axis.textContent = measure(totals[side][index]));
      pad.active.textContent = displayed.some(entry => entry[3] > .004)
        ? `${displayed.filter(entry => entry[3] >= .02).length} of 77 active`
        : "No contact";
      paintMap(pad.map, displayed);
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

  function makeSummaryPad(container, coords, side) {
    const panel = document.createElement("figure");
    const caption = document.createElement("figcaption");
    const name = document.createElement("span");
    const total = document.createElement("strong");
    name.textContent = sides[side];
    caption.append(name, total);
    panel.append(caption);
    const map = makeMap(panel, coords, side, false);
    container.append(panel);
    return {map, total};
  }

  function setSummaryPad(pad, frame, side) {
    pad.total.textContent = `${measure(frame.totals[side][2])} N`;
    paintMap(pad.map, frame.estimate[side]);
  }

  function drawComparison(card, data) {
    for (const stage of card.querySelectorAll("[data-compare-stage]")) {
      const second = data.jumps[Number(stage.dataset.compareStage)];
      const frame = nearest(data.frames, second);
      const group = stage.querySelector(".comparison-pads");
      sides.forEach((_, side) => setSummaryPad(makeSummaryPad(group, data.coords, side), frame, side));
    }
  }

  function setupSynchronizedComparison() {
    const movie = document.querySelector("#compare-video");
    const playAll = document.querySelector("#compare-play");
    const speed = document.querySelector("#compare-speed");
    const repeat = document.querySelector("#compare-repeat");
    const slider = document.querySelector("#compare-seek");
    const clock = document.querySelector("#compare-time");
    const status = document.querySelector("#compare-status");
    const markers = [...document.querySelectorAll("[data-compare-jump]")];
    const groups = [...document.querySelectorAll("[data-live-trial]")];
    const controls = [playAll, speed, repeat, slider, ...markers];
    const labels = {Start: "Before closure", Closing: "Closing jaws", "Jaws closed": "Jaws closed",
      "Lift starts": "Lifting", "Late lift": "Late lift", End: "Comparison complete"};
    let timeline;
    let views;
    let shown = -1;
    let callback;
    let inView = false;
    let allowAutoplay = true;
    const videoFrameCallback = typeof movie.requestVideoFrameCallback === "function";

    function show(index) {
      if (!timeline) return;
      const frameIndex = Math.min(Number(slider.max), Math.max(0, index));
      if (frameIndex === shown) return;
      shown = frameIndex;
      const [phase] = Object.entries(timeline.milestones).findLast(([, start]) => start <= frameIndex);
      status.textContent = labels[phase];
      const elapsed = (frameIndex / timeline.fps).toFixed(2);
      const duration = (Number(slider.max) / timeline.fps).toFixed(2);
      clock.textContent = `${elapsed} / ${duration} s`;
      slider.value = frameIndex;
      if (movie.paused) playAll.textContent = frameIndex === Number(slider.max) ? "Replay all" : "Play all";
      slider.setAttribute("aria-valuetext", `${elapsed} of ${duration} seconds, ${labels[phase]}`);
      for (const marker of markers) {
        marker.setAttribute("aria-pressed", String(frameIndex === timeline.milestones[marker.dataset.compareJump]));
      }
      for (const view of views) {
        const sample = view.frames[frameIndex];
        if (sample === view.shown) continue;
        view.shown = sample;
        view.pads.forEach((pad, side) => setSummaryPad(pad, sample, side));
      }
    }

    function updateFromClock() {
      if (timeline && !movie.seeking) show(Math.floor(movie.currentTime * timeline.fps + .0001));
    }

    function jump(index) {
      allowAutoplay = false;
      movie.pause();
      const position = index / timeline.fps;
      if (Math.abs(movie.currentTime - position) < .001 && !movie.seeking) show(index);
      else movie.currentTime = position;
    }

    function drawFrame(_, metadata) {
      if (movie.paused || movie.ended) return;
      if (videoFrameCallback) {
        if (!movie.seeking) show(Math.round(metadata.mediaTime * timeline.fps));
        callback = movie.requestVideoFrameCallback(drawFrame);
      } else {
        updateFromClock();
        callback = requestAnimationFrame(drawFrame);
      }
    }

    function applySpeed() {
      const rate = Number(speed.value);
      movie.defaultPlaybackRate = rate;
      movie.playbackRate = rate;
    }

    function applyRepeat() {
      movie.loop = repeat.checked;
    }

    function startPlayback(automatic = false) {
      if (movie.ended || shown === Number(slider.max)) movie.currentTime = 0;
      movie.play().catch(error => {
        if (error.name === "AbortError" || (automatic && error.name === "NotAllowedError")) return;
        showError();
      });
    }

    function autoplayWhenVisible() {
      if (inView && allowAutoplay && !reducedMotion.matches && !playAll.disabled && movie.paused) {
        startPlayback(true);
      }
    }

    function enableControls() {
      if (!timeline || movie.readyState < HTMLMediaElement.HAVE_METADATA) return;
      applySpeed();
      applyRepeat();
      for (const control of controls) control.disabled = false;
      autoplayWhenVisible();
    }

    function showError() {
      for (const control of controls) control.disabled = true;
      status.textContent = "Synchronized replay unavailable. The key frames are below.";
    }

    Promise.all([
      fetch("data/touch-synchronized.json").then(response => {
        if (!response.ok) throw new Error("Timeline unavailable");
        return response.json();
      }),
      ...groups.map(group => recording(group.dataset.liveTrial))
    ]).then(([aligned, ...recordings]) => {
      views = groups.map((group, index) => {
        const data = recordings[index];
        const samples = new Map(data.frames.map(frame => [frame.s, frame]));
        const frames = aligned.times[group.dataset.liveTrial].map(second => samples.get(second));
        if (frames.length !== aligned.milestones.End + 1 || frames.some(frame => !frame)) {
          throw new Error("Timeline is missing a recorded frame");
        }
        return {frames, pads: sides.map((_, side) => makeSummaryPad(group, data.coords, side))};
      });
      timeline = aligned;
      slider.max = timeline.milestones.End;
      show(0);
      enableControls();
    }).catch(showError);

    movie.addEventListener("loadedmetadata", enableControls);
    movie.addEventListener("error", showError);
    movie.addEventListener("seeked", updateFromClock);
    movie.addEventListener("ended", () => {
      allowAutoplay = false;
      updateFromClock();
      playAll.textContent = "Replay all";
    });
    movie.addEventListener("play", () => {
      video.pause();
      playAll.textContent = "Pause all";
      callback = videoFrameCallback ? movie.requestVideoFrameCallback(drawFrame) : requestAnimationFrame(drawFrame);
    });
    movie.addEventListener("pause", () => {
      playAll.textContent = shown === Number(slider.max) ? "Replay all" : "Play all";
      if (callback === undefined) return;
      if (videoFrameCallback) movie.cancelVideoFrameCallback(callback);
      else cancelAnimationFrame(callback);
      callback = undefined;
    });
    video.addEventListener("play", () => {
      allowAutoplay = false;
      movie.pause();
    });
    for (const marker of markers) marker.addEventListener("click", () => jump(timeline.milestones[marker.dataset.compareJump]));
    slider.addEventListener("input", () => jump(Number(slider.value)));
    speed.addEventListener("change", applySpeed);
    repeat.addEventListener("change", applyRepeat);
    playAll.addEventListener("click", () => {
      allowAutoplay = movie.paused;
      if (allowAutoplay) startPlayback();
      else movie.pause();
    });
    reducedMotion.addEventListener("change", () => {
      if (reducedMotion.matches) movie.pause();
      else autoplayWhenVisible();
    });
    const visibleFraction = .62;
    new IntersectionObserver(([entry]) => {
      inView = entry.intersectionRatio >= visibleFraction;
      if (inView) autoplayWhenVisible();
      else movie.pause();
    }, {threshold: visibleFraction}).observe(movie);
  }

  async function selectCase(id) {
    const version = ++selectionVersion;
    video.pause();
    pendingTime = undefined;
    activeRecording = undefined;
    currentFrame = undefined;
    focusedPoint = undefined;
    pointOutput.hidden = true;
    play.disabled = restart.disabled = seek.disabled = true;
    seek.value = 0;
    time.textContent = "0.00 s";
    for (const jump of jumps) {
      jump.disabled = true;
      jump.setAttribute("aria-pressed", "false");
    }
    phase.textContent = "Loading recording";
    caption.textContent = cases[id];
    caseSelect.value = id;
    padRow.setAttribute("aria-busy", "true");
    try {
      const data = await recording(id);
      if (version !== selectionVersion) return;
      activeRecording = data;
      padRow.replaceChildren();
      padViews = sides.map((_, i) => makePad(i, data.coords));
      padRow.removeAttribute("aria-busy");
      seek.max = (data.frames.at(-1).s - data.videoStart).toFixed(2);
      for (const jump of jumps) jump.disabled = false;
      play.disabled = restart.disabled = seek.disabled = false;
      for (const link of observations) {
        const url = `media/observations/${id}-${link.dataset.feed}.png`;
        link.href = url;
        link.querySelector("img").src = url;
      }
      download.href = `data/touch-${id}.json?v=compare`;
      download.download = `touch-${id}.json`;
      const url = `media/touch-${id}.mp4?v=compare`;
      if (video.getAttribute("src") !== url) {
        video.poster = `media/touch-${id}.jpg?v=compare`;
        video.src = url;
        video.load();
      }
      jumpTo(data.videoStart);
    } catch {
      if (version === selectionVersion) {
        padRow.replaceChildren();
        padRow.removeAttribute("aria-busy");
        phase.textContent = "Recording unavailable. Select a trial to try again.";
      }
    }
  }

  for (const card of comparisons) {
    const id = card.dataset.comparison;
    recording(id).then(data => drawComparison(card, data)).catch(() => {
      for (const panel of card.querySelectorAll(".comparison-pads")) panel.textContent = "Force recording unavailable.";
    });
    card.querySelector("[data-touch-case]").addEventListener("click", () => {
      selectCase(id);
      detailTitle.focus({preventScroll: true});
      document.querySelector("#touch-detail").scrollIntoView({block: "start"});
    });
  }
  caseSelect.addEventListener("change", () => selectCase(caseSelect.value));
  for (const [index, button] of jumps.entries()) {
    button.addEventListener("click", () => jumpTo(activeRecording.jumps[index]));
  }
  mapMode.addEventListener("change", () => {
    if (currentFrame) render(currentFrame.s, true);
  });
  seek.addEventListener("input", () => jumpTo(activeRecording.videoStart + Number(seek.value)));
  restart.addEventListener("click", () => jumpTo(activeRecording.videoStart));
  play.addEventListener("click", () => {
    if (!video.paused) {
      video.pause();
      return;
    }
    if (video.ended) video.currentTime = 0;
    const version = selectionVersion;
    video.play().catch(error => {
      if (error.name !== "AbortError" && version === selectionVersion) phase.textContent = "The recording could not be played.";
    });
  });
  video.addEventListener("error", () => {
    if (activeRecording) phase.textContent = "The recording could not be played.";
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
  video.addEventListener("ended", updateFromVideo);
  if (!video.requestVideoFrameCallback) video.addEventListener("timeupdate", updateFromVideo);
  new IntersectionObserver(([entry]) => {
    if (!entry.isIntersecting) video.pause();
  }).observe(document.querySelector(".touch-layout"));
  setupSynchronizedComparison();
  selectCase("offset");
}
