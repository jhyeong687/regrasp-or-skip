"use strict";

{
  const cases = {
    offset: "턱이 닫히는 동안 왼쪽 패드에만 접촉력이 기록됐습니다.",
    slip: "턱이 닫힐 때는 양쪽에 접촉력이 기록됐고, 물체가 미끄러진 뒤 두 패드의 신호가 사라졌습니다."
  };
  const video = document.querySelector("#touch-video");
  const caption = document.querySelector("#touch-caption");
  const phase = document.querySelector("#touch-phase");
  const time = document.querySelector("#touch-time");
  const choices = [...document.querySelectorAll("[data-touch-case]")];
  const jumps = [...document.querySelectorAll("[data-touch-time]")];
  const pads = [document.querySelector("#pad-left"), document.querySelector("#pad-right")];
  const totals = [document.querySelector("#touch-left"), document.querySelector("#touch-right")];
  const chart = document.querySelector("#touch-chart");
  const dataCache = new Map();
  const amber = "#ffbc70";
  const mint = "#7ce5d4";
  const [leftColor, rightColor] = [amber, "#c7a4ff"];
  let selected;
  let recording;
  let drawnFrame;
  let frameCallback;
  let pendingSeek;

  function canvasView(canvas) {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const scale = Math.min(window.devicePixelRatio || 1, 2);
    const pixelWidth = Math.round(width * scale);
    const pixelHeight = Math.round(height * scale);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    ctx.clearRect(0, 0, width, height);
    return {ctx, width, height};
  }

  function drawPad(canvas, coords, forces) {
    const {ctx, width, height} = canvasView(canvas);
    const {minX, maxX, minY, maxY, points} = coords;
    const scale = Math.min((width - 34) / (maxX - minX), (height - 28) / (maxY - minY));
    const project = ([x, y]) => [
      width / 2 + (x - (minX + maxX) / 2) * scale,
      height / 2 - (y - (minY + maxY) / 2) * scale
    ];

    ctx.fillStyle = "#62546e";
    for (const point of points) {
      const [x, y] = project(point);
      ctx.beginPath();
      ctx.arc(x, y, 2.3, 0, Math.PI * 2);
      ctx.fill();
    }

    for (const [index, , , normal] of forces) {
      if (normal < 0.009) continue;
      const [x, y] = project(points[index]);
      const intensity = Math.min(1, Math.sqrt(normal / 0.85));
      const radius = 2.8 + 5.5 * intensity;
      ctx.beginPath();
      ctx.arc(x, y, radius + 2.5, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(255, 166, 82, ${0.24 * intensity})`;
      ctx.fill();
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(255, 188, 112, ${0.60 + 0.40 * intensity})`;
      ctx.fill();
    }

    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    for (const [index, fx, fy] of forces) {
      const shear = Math.hypot(fx, fy);
      if (shear < 0.007) continue;
      const [x, y] = project(points[index]);
      const length = Math.min(25, 7 + 48 * Math.sqrt(shear));
      const ux = fx / shear;
      const uy = -fy / shear;
      const ex = x + ux * length;
      const ey = y + uy * length;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(ex, ey);
      ctx.moveTo(ex - 5.5 * ux + 3.5 * uy, ey - 5.5 * uy - 3.5 * ux);
      ctx.lineTo(ex, ey);
      ctx.lineTo(ex - 5.5 * ux - 3.5 * uy, ey - 5.5 * uy + 3.5 * ux);
      ctx.strokeStyle = "#191422";
      ctx.lineWidth = 4.5;
      ctx.stroke();
      ctx.strokeStyle = mint;
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }

  function drawChart(current, data) {
    const {ctx, width, height} = canvasView(chart);
    const samples = data.frames.filter(frame => frame.s >= data.videoStart);
    const left = 31;
    const right = width - 12;
    const top = 13;
    const bottom = height - 20;
    const x = second => left + (second - data.videoStart) /
      (samples.at(-1).s - data.videoStart) * (right - left);
    const y = force => bottom - (bottom - top) * Math.min(2, Math.max(0, force)) / 2;

    const lift = samples.find(frame => frame.phase === "시험 들어올리기");
    if (lift) {
      ctx.fillStyle = "#282138";
      ctx.fillRect(x(lift.s), top, right - x(lift.s), bottom - top);
    }

    ctx.font = "10px system-ui, sans-serif";
    ctx.fillStyle = "#c4b9d2";
    ctx.textAlign = "right";
    for (const value of [0, 1, 2]) {
      const row = y(value);
      ctx.beginPath();
      ctx.moveTo(left, row);
      ctx.lineTo(right, row);
      ctx.strokeStyle = "#41364d";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillText(value === 2 ? "2 N" : String(value), left - 6, row + 3);
    }

    ctx.textAlign = "center";
    for (const second of [1, 2, 3, 4]) ctx.fillText(`${second}초`, x(second), height - 4);

    for (const [pad, color] of [leftColor, rightColor].entries()) {
      ctx.beginPath();
      for (const [index, frame] of samples.entries()) {
        if (index === 0) ctx.moveTo(x(frame.s), y(frame.totals[pad]));
        else ctx.lineTo(x(frame.s), y(frame.totals[pad]));
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.9;
      ctx.setLineDash(pad === 1 ? [4, 3] : []);
      ctx.stroke();
    }

    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(x(current.s), top);
    ctx.lineTo(x(current.s), bottom);
    ctx.strokeStyle = "#f7f2fb";
    ctx.lineWidth = 1.1;
    ctx.stroke();
  }

  function render(force = false, mediaTime = video.currentTime) {
    if (!recording) return;
    const simulationTime = mediaTime + recording.videoStart;
    const frame = recording.frames.reduce((nearest, sample) =>
      Math.abs(sample.s - simulationTime) < Math.abs(nearest.s - simulationTime) ? sample : nearest);
    if (!force && drawnFrame === frame) return;
    drawnFrame = frame;
    phase.textContent = frame.phase;
    time.textContent = `${frame.s.toFixed(2)}초`;
    pads.forEach((canvas, index) => {
      drawPad(canvas, recording.geometry, frame.active[index]);
      totals[index].textContent = `${Math.max(0, frame.totals[index]).toFixed(2)} N`;
    });
    drawChart(frame, recording);
  }

  async function selectCase(id) {
    selected = id;
    pendingSeek = undefined;
    delete video.dataset.inspecting;
    recording = undefined;
    drawnFrame = undefined;
    phase.textContent = "신호 불러오는 중";
    time.textContent = "—";
    for (const choice of choices) choice.setAttribute("aria-pressed", String(choice.dataset.touchCase === id));
    for (const jump of jumps) {
      jump.disabled = true;
      jump.setAttribute("aria-pressed", "false");
    }
    for (const total of totals) total.textContent = "— N";
    for (const canvas of [...pads, chart]) canvasView(canvas);
    caption.textContent = cases[id];

    if (video.getAttribute("src") !== `media/touch-${id}.mp4`) {
      video.poster = `media/touch-${id}.jpg`;
      video.src = `media/touch-${id}.mp4`;
      autoVisible.delete(video);
      queueUpdate();
    }

    try {
      if (!dataCache.has(id)) {
        dataCache.set(id, fetch(`data/touch-${id}.json`).then(response => {
          if (!response.ok) throw new Error("Tactile recording unavailable");
          return response.json();
        }));
      }
      const data = await dataCache.get(id);
      if (selected !== id) return;
      const xs = data.coords.map(([x]) => x);
      const ys = data.coords.map(([, y]) => y);
      recording = {
        ...data,
        geometry: {
          minX: Math.min(...xs), maxX: Math.max(...xs),
          minY: Math.min(...ys), maxY: Math.max(...ys), points: data.coords
        }
      };
      for (const jump of jumps) jump.disabled = false;
      render(true);
    } catch {
      dataCache.delete(id);
      if (selected === id) {
        phase.textContent = "촉각 기록을 불러오지 못했습니다.";
        caption.textContent = "영상은 재생할 수 있습니다. 사례 버튼을 다시 누르면 기록을 다시 불러옵니다.";
      }
    }
  }

  function onVideoFrame(_, {mediaTime}) {
    render(false, mediaTime);
    frameCallback = video.requestVideoFrameCallback(onVideoFrame);
  }

  for (const choice of choices) {
    choice.addEventListener("click", () => {
      if (selected !== choice.dataset.touchCase || !recording) selectCase(choice.dataset.touchCase);
    });
  }
  for (const jump of jumps) {
    jump.addEventListener("click", () => {
      if (!recording) return;
      const mediaTime = Number(jump.dataset.touchTime) - recording.videoStart;
      video.dataset.inspecting = "true";
      video.pause();
      if (video.readyState >= HTMLMediaElement.HAVE_METADATA) {
        video.currentTime = mediaTime;
      } else {
        pendingSeek = mediaTime;
      }
      for (const item of jumps) item.setAttribute("aria-pressed", String(item === jump));
      render(true, mediaTime);
    });
  }
  if (video.requestVideoFrameCallback) {
    video.addEventListener("play", () => {
      if (frameCallback === undefined) frameCallback = video.requestVideoFrameCallback(onVideoFrame);
    });
    video.addEventListener("pause", () => {
      if (frameCallback !== undefined) video.cancelVideoFrameCallback(frameCallback);
      frameCallback = undefined;
    });
  } else {
    video.addEventListener("timeupdate", () => render());
  }
  video.addEventListener("loadedmetadata", () => {
    if (pendingSeek === undefined) return;
    video.currentTime = pendingSeek;
    pendingSeek = undefined;
  });
  video.addEventListener("play", () => {
    if (video.paused) return;
    delete video.dataset.inspecting;
    for (const jump of jumps) jump.setAttribute("aria-pressed", "false");
  });
  video.addEventListener("seeked", () => render());
  const resizeObserver = new ResizeObserver(() => render(true));
  for (const canvas of [...pads, chart]) resizeObserver.observe(canvas);
  selectCase("offset");
}
