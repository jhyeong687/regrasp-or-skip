"use strict";

const slides = [...document.querySelectorAll(".slide")];
const videos = [...document.querySelectorAll("video:not([data-manual-playback])")];
const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
const prevButton = document.querySelector("#previous");
const nextButton = document.querySelector("#next");
const fullscreen = document.querySelector("#fullscreen");
const autoVisible = new WeakMap();
let active = 0;
let pendingUpdate = false;

function visibleVideo(video) {
  if (video.closest("[hidden]") || video.closest("details:not([open])")) return false;
  const bounds = video.getBoundingClientRect();
  const mastBottom = document.querySelector(".mast").getBoundingClientRect().bottom;
  const footerTop = document.querySelector(".deck-footer").getBoundingClientRect().top;
  const visibleHeight = Math.min(bounds.bottom, footerTop) - Math.max(bounds.top, mastBottom);
  return bounds.height > 0 && visibleHeight >= bounds.height * 0.62;
}

function updatePlayback() {
  for (const video of videos) {
    const visible = visibleVideo(video);
    if (visible && !reducedMotion.matches && !autoVisible.get(video)) {
      video.muted = true;
      video.play().catch(() => {});
    }
    if (!visible && autoVisible.get(video)) video.pause();
    autoVisible.set(video, visible);
  }
}

function updatePosition() {
  pendingUpdate = false;
  const readingLine = window.innerHeight * 0.41;
  const detected = slides.findLastIndex(slide => slide.getBoundingClientRect().top <= readingLine);
  active = Math.max(0, detected);
  document.querySelector("#slide-number").textContent =
    `${String(active + 1).padStart(2, "0")} / ${String(slides.length).padStart(2, "0")}`;
  document.querySelector("#slide-name").textContent = slides[active].dataset.title;
  document.querySelector("#progress").style.width = `${(active + 1) / slides.length * 100}%`;
  prevButton.disabled = active === 0;
  nextButton.disabled = active === slides.length - 1;
  for (const link of document.querySelectorAll(".mast nav a")) {
    if (link.hash === "#" + slides[active].id) link.setAttribute("aria-current", "location");
    else link.removeAttribute("aria-current");
  }
  updatePlayback();
}

function queueUpdate() {
  if (pendingUpdate) return;
  pendingUpdate = true;
  requestAnimationFrame(updatePosition);
}

function goTo(index) {
  const target = slides[Math.max(0, Math.min(slides.length - 1, index))];
  target.scrollIntoView({behavior: reducedMotion.matches ? "instant" : "smooth"});
  history.replaceState(null, "", "#" + target.id);
}

prevButton.addEventListener("click", () => goTo(active - 1));
nextButton.addEventListener("click", () => goTo(active + 1));
document.addEventListener("keydown", event => {
  if (event.altKey || event.metaKey || event.ctrlKey || event.shiftKey ||
      event.target.closest("button, a, input, select, textarea, video, summary, [contenteditable]")) return;
  const keys = { ArrowLeft: -1, PageUp: -1, ArrowRight: 1, PageDown: 1 };
  if (keys[event.key]) {
    event.preventDefault();
    goTo(active + keys[event.key]);
  }
});
window.addEventListener("scroll", queueUpdate, {passive: true});
window.addEventListener("resize", queueUpdate, {passive: true});
for (const details of document.querySelectorAll("details")) details.addEventListener("toggle", queueUpdate);
if (reducedMotion.matches) videos.find(video => video.autoplay)?.pause();
reducedMotion.addEventListener("change", () => {
  if (reducedMotion.matches) for (const video of videos) video.pause();
  else for (const video of videos) autoVisible.delete(video);
  queueUpdate();
});

if (document.fullscreenEnabled && document.documentElement.requestFullscreen) {
  fullscreen.hidden = false;
  fullscreen.addEventListener("click", async () => {
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    } catch (error) {
      fullscreen.title = "Full screen is unavailable in this browser.";
    }
  });
  document.addEventListener("fullscreenchange", () => {
    fullscreen.textContent = document.fullscreenElement ? "Exit full screen" : "Full screen";
    queueUpdate();
  });
}

const stages = {
  clear: "Clear", marker_occlusion: "Front blocked", one_side: "One side",
  slit: "Narrow slit", full_occlusion: "All probe views blocked"
};
const shapes = {
  beveled_block: "Beveled block", rounded_rectangle: "Rounded rectangular prism", tapered_block: "Tapered block"
};
const actions = {position_correction: "Position correction", grip_reinforcement: "Grip reinforcement"};
const media = document.querySelector("#library-video");
const caption = document.querySelector("#clip-detail");
const download = document.querySelector("#clip-download");
const stageFilter = document.querySelector("#stage-filter");
const outcomeFilter = document.querySelector("#outcome-filter");
const clipList = document.querySelector("#clips");
const clipCount = document.querySelector("#clip-count");
let allClips = [];
let selectedTrial = 15;

function resultText(row) {
  if (row.tactile_success && row.control_success) return "Both placed";
  if (row.tactile_success) return "Placed with PaXini only";
  if (row.control_success) return "Placed without PaXini only";
  return "Neither placed";
}

function selectClip(row, userAction = false) {
  selectedTrial = row.trial;
  const id = String(row.trial).padStart(2, "0");
  const label = `Trial ${id} · ${stages[row.condition]}`;
  const context = `${shapes[row.shape]} · ${row.fault === "off_center" ? "Off-center" : "Low friction"} · ${resultText(row)}`;
  caption.querySelector("strong").textContent = label;
  caption.querySelector("span").textContent = `${context} · ${actions[row.tactile_action]} / ${actions[row.control_action]}`;
  const src = row.file + "?v=hd";
  if (media.getAttribute("src") !== src) {
    media.poster = row.file.replace(/\.mp4$/, ".jpg?v=hd");
    media.src = src;
    autoVisible.delete(media);
  }
  download.href = src;
  for (const child of clipList.querySelectorAll("button")) {
    child.setAttribute("aria-pressed", String(Number(child.dataset.trial) === selectedTrial));
  }
  if (userAction && visibleVideo(media) && !reducedMotion.matches) media.play().catch(() => {});
  queueUpdate();
}

function updateLibrary() {
  const result = allClips.filter(row => {
    const stageMatches = !stageFilter.value || row.condition === stageFilter.value;
    const winnerMatches = !outcomeFilter.value ||
      (outcomeFilter.value === "tactile" && row.tactile_success && !row.control_success) ||
      (outcomeFilter.value === "both" && row.tactile_success && row.control_success);
    return stageMatches && winnerMatches;
  });
  clipCount.textContent = result.length ? `${result.length} pairs` : "No pairs match these filters.";
  const selected = result.find(row => row.trial === selectedTrial) || result[0];
  clipList.replaceChildren(...result.map(row => {
    const item = document.createElement("button");
    item.type = "button";
    item.dataset.trial = row.trial;
    const label = document.createElement("span");
    label.textContent = `${String(row.trial).padStart(2, "0")} · ${shapes[row.shape]} · ${stages[row.condition]}`;
    const detail = document.createElement("small");
    detail.textContent = `${resultText(row)} · ${row.tactile_wall_s.toFixed(2)} s / ${row.control_wall_s.toFixed(2)} s`;
    item.append(label, detail);
    item.addEventListener("click", () => selectClip(row, true));
    return item;
  }));
  if (selected) selectClip(selected);
}
stageFilter.addEventListener("change", updateLibrary);
outcomeFilter.addEventListener("change", updateLibrary);

fetch("data/clips.json")
  .then(response => {
    if (!response.ok) throw new Error("Clip index unavailable");
    return response.json();
  })
  .then(rows => { allClips = rows; updateLibrary(); })
  .catch(() => { clipCount.textContent = "The video index is unavailable. The trial CSV lists the video paths."; });

queueUpdate();
