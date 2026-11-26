"use strict";

const slides = [...document.querySelectorAll(".slide")];
const videos = [...document.querySelectorAll("video")];
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
      fullscreen.title = "이 브라우저에서 전체 화면을 열 수 없습니다.";
    }
  });
  document.addEventListener("fullscreenchange", () => {
    fullscreen.textContent = document.fullscreenElement ? "전체 화면 닫기" : "전체 화면";
    queueUpdate();
  });
}

for (const switcher of document.querySelectorAll("[data-switch]")) {
  const tabs = [...switcher.querySelectorAll("[role=tab]")];
  function select(tab, focus = false) {
    for (const item of tabs) {
      const selected = item === tab;
      item.setAttribute("aria-selected", String(selected));
      item.tabIndex = selected ? 0 : -1;
      document.getElementById(item.getAttribute("aria-controls")).hidden = !selected;
    }
    if (focus) tab.focus();
    queueUpdate();
  }
  select(tabs[0]);
  tabs.forEach((tab, index) => {
    tab.addEventListener("click", () => select(tab));
    tab.addEventListener("keydown", event => {
      const keys = {ArrowLeft: (index + tabs.length - 1) % tabs.length,
                    ArrowRight: (index + 1) % tabs.length, Home: 0, End: tabs.length - 1};
      if (Object.hasOwn(keys, event.key)) {
        event.preventDefault();
        select(tabs[keys[event.key]], true);
      }
    });
  });
}

const stages = {
  clear: "추가 가림 없음", marker_occlusion: "정면 턱 표식 가림", one_side: "한쪽 RGB",
  slit: "좁은 틈", full_occlusion: "양쪽 측면 RGB 가림"
};
const shapes = {
  beveled_block: "모서리 절삭 블록", rounded_rectangle: "둥근 직사각기둥", tapered_block: "테이퍼 블록"
};
const actions = {position_correction: "위치 보정", grip_reinforcement: "그립 보강"};
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
  if (row.tactile_success && row.control_success) return "둘 다 안착";
  if (row.tactile_success) return "촉각 제공만 안착";
  if (row.control_success) return "촉각 제외만 안착";
  return "양쪽 모두 실패";
}

function selectClip(row, userAction = false) {
  selectedTrial = row.trial;
  const id = String(row.trial).padStart(2, "0");
  const label = `${id}번 · ${stages[row.condition]}`;
  const context = `${shapes[row.shape]} · ${row.fault === "off_center" ? "편심" : "저마찰"} · ${resultText(row)}`;
  caption.querySelector("strong").textContent = label;
  caption.querySelector("span").textContent = `${context} · ${actions[row.tactile_action]} / ${actions[row.control_action]}`;
  if (media.getAttribute("src") !== row.file) {
    media.removeAttribute("poster");
    media.src = row.file;
    autoVisible.delete(media);
  }
  download.href = row.file;
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
  clipCount.textContent = result.length ? `${result.length}쌍` : "이 조건의 영상이 없습니다.";
  const selected = result.find(row => row.trial === selectedTrial) || result[0];
  clipList.replaceChildren(...result.map(row => {
    const item = document.createElement("button");
    item.type = "button";
    item.dataset.trial = row.trial;
    const label = document.createElement("span");
    label.textContent = `${String(row.trial).padStart(2, "0")} · ${shapes[row.shape]} · ${stages[row.condition]}`;
    const detail = document.createElement("small");
    detail.textContent = `${resultText(row)} · ${row.tactile_wall_s.toFixed(2)}초 / ${row.control_wall_s.toFixed(2)}초`;
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
  .catch(() => { clipCount.textContent = "영상 목록을 불러오지 못했습니다. 전체 목록 CSV에서 파일을 확인할 수 있습니다."; });

queueUpdate();
