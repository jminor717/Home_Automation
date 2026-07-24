//! NOTE TO SELF: vibe code slop 
//! PROCEED WITH CAUTION

import "https://unpkg.com/wired-card@0.8.1/wired-card.js?module";
import {
    LitElement,
    html,
    css,
} from "https://unpkg.com/lit-element@2.0.1/lit-element.js?module";

function loadCSS(url) {
    const link = document.createElement("link");
    link.type = "text/css";
    link.rel = "stylesheet";
    link.href = url;
    document.head.appendChild(link);
}

loadCSS("https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700;800&display=swap");

class SetpointGraphCard extends LitElement {
    static get properties() {
        return {
            hass: {},
            config: {},
        };
    }

    static get styles() {
        return css`
      
      .card { border-radius: 12px; padding: 18px; }
      .local-header { margin: 0 0 8px; font-size: 1.25rem; }
      .text-block-p { margin: 0 0 12px; }
      .graph-wrapper { position: relative; }
      .graph-info { display: grid; gap: 12px; margin-top: 14px; grid-template-columns: repeat(auto-fit, minmax(120px,1fr)); }
      .setpoint-chip { display:flex; flex-direction:column; gap:4px; padding:10px 12px; border:1px solid #e8eef7; border-radius:10px; text-align:center }
      .setpoint-chip.active { border-color:#4965ff; transform: translateY(-2px); }
      .value { font-size:1.05rem; font-weight:700; }
      .label { font-size:0.92rem; }
      .time-editor { display:flex; gap:4px; align-items:center; font-size:0.85rem }
      .time-editor input { width:2em; padding:0px; border:1px solid #000000; border-radius:4px; text-align:center }
      .time-separator { font-weight:600 }
      .graph-svg { width:100%; height:auto; touch-action:none; user-select:none }
      .graph-svg:active { cursor:grabbing }
      .button-row { margin-top:14px; display:flex; justify-content:space-between; gap:8px; flex-wrap:wrap }
      .local-button { border:none; border-radius:999px; padding:10px 14px; font-weight:700; }
      .not-found { background-color: yellow; font-family: sans-serif; font-size: 14px; padding: 8px; }
    `;
    }

    startingProfileName = "Default";
    startingSetpoints = 3;
    constructor() {
        super();
        this.numSetpoints = this.startingSetpoints;
        this.minTemp = 50;
        this.maxTemp = 110;
        this.margin = { top: 36, right: 30, bottom: 42, left: 48 };
        this.width = 760;
        this.height = 420;
        this.contentWidth = this.width - this.margin.left - this.margin.right;
        this.contentHeight = this.height - this.margin.top - this.margin.bottom;
        // this.setpoints = [];
        this.activeIndex = null;
        this.durationSeconds = 6 * 3600;
        this.graphedPoints = [];
        this.currentProfileData = this.defaultProfile();
        this.allProfiles = [];
        this.profileIsActive = false;
    }

    defaultProfile = () => ({ profileName: this.startingProfileName, setpoints: this.generateSetpoints(this.startingSetpoints) });

    render() {

        const stateObj = this.hass.states[this.profileStorage];
        const activeObj = this.hass.states[this.activeProfile];
        // console.log(this.allProfiles);
        let profileNames = this.allProfiles && this.allProfiles.map ? this.allProfiles.map(p => p.profileName) : [];
        profileNames.push(this.startingProfileName);
        // filter out non strings
        profileNames = profileNames.filter(name => name && typeof name === 'string');
        return html`
      <ha-card>
        <div class="card">
          <h1 class="local-header">Temperature Setpoint Schedule</h1>
          <p class="text-block-p">Drag any point to change its setpoint. The graph shows temperature across a schedule.</p>

          <div style="margin-bottom:12px; display:flex; gap:12px; align-items:center; flex-wrap:wrap;">
            <div style="display:flex; gap:12px; align-items:center;">
              <label style="font-weight:600; color:${this.textColor};">Setpoints:</label>
              <input id="numSetpoints" type="number" min="2" max="12" .value="${this.numSetpoints}" @change="${this.onNumSetpointsChange}" style="width:60px; padding:8px; border:2px solid #e8eef7; border-radius:8px;" />
            </div>
            <div style="display:flex; gap:12px; align-items:center;">
              <label style="font-weight:600; color:${this.textColor};">Duration:</label>
              <div style="display:flex; gap:4px; align-items:center;">
                <input id="durationHours" type="number" min="0" max="23" .value="${Math.floor(this.durationSeconds / 3600)}" @change="${this.onDurationChange}" style="width:50px; padding:8px; border:2px solid #e8eef7; border-radius:8px;" />
                <span style="font-weight:600; color:${this.textColor};">h</span>
                <input id="durationMinutes" type="number" min="0" max="59" .value="${Math.floor((this.durationSeconds % 3600) / 60)}" @change="${this.onDurationChange}" style="width:50px; padding:8px; border:2px solid #e8eef7; border-radius:8px;" />
                <span style="font-weight:600; color:${this.textColor};">m</span>
              </div>
            </div>
          </div>

          <select id="profileSelect" @change="${this.profileSelected}" style="margin-bottom:12px;">
            ${profileNames.map(name => html`
            <option value="${name}" ?selected="${name === this.currentProfileData.profileName}">
                ${name}
            </option>
            `)}
          </select>
          <div>Profile Name: 
              <input type="text" id="newProfileName" placeholder="New profile name" .value="${this.currentProfileData.profileName}" 
                @change="${this.onProfileNameChange}"/>
          </div>

          <div class="graph-wrapper">
            <svg class="graph-svg" viewBox="0 0 ${this.width} ${this.height}" aria-label="Temperature setpoint graph"></svg>
          </div>

          <div class="graph-info" id="setpointInfo"></div>

          <div class="button-row">
          ${stateObj && activeObj ?
                html`<button class="local-button" id="saveBtn" @click="${this.saveToEntity}">Save profile</button>
             <button class="local-button" id="resetBtn" @click="${this.DeleteProfile}" style="background-color: #f44336; color: white;"
             >Delete profile</button>
             <button class="local-button" id="setActiveBtn" @click="${this.activateProfile}"
             style="background-color: ${this.profileIsActive ? '#03a009' : '#8a8100'}; color: white;"
             >Activate profile</button>
             `:
                html`<div class="not-found">Entity ${this.profileStorage} not found.</div>`
            }
          </div>
        </div>
      </ha-card>
    `;
    }
    firstUpdated() {
        // Element refs
        this.svg = this.shadowRoot.querySelector('.graph-svg');
        this.info = this.shadowRoot.getElementById('setpointInfo');

        // Pointer events
        this.svg.addEventListener('pointerdown', this._onPointerDown.bind(this));
        this.svg.addEventListener('pointermove', this._onPointerMove.bind(this));
        this.svg.addEventListener('pointerup', this._onPointerUp.bind(this));
        this.svg.addEventListener('pointerleave', this._onPointerLeave.bind(this));

        // Initialize
        this.currentProfileData.setpoints = this.generateSetpoints(this.numSetpoints);

        const rootStyles = window.getComputedStyle(document.documentElement);
        this.mainColor = rootStyles.getPropertyValue('--primary-color');
        this.textColor = rootStyles.getPropertyValue('--primary-text-color');
        this.secondaryTextColor = rootStyles.getPropertyValue('--secondary-text-color');
        this.accentColor = rootStyles.getPropertyValue('--accent-color');
        console.log(this.hass.states["sensor.store_temp_profiles"], this.mainColor, this.textColor, this.secondaryTextColor, this.accentColor, this.config, this.hass);

        this.setCurrentProfile(this.defaultProfile());

        this.loadProfiles();
        this.profileIsActive = this.currentProfileIsActive();
        this.renderGraph();
    }

    setConfig(config) {
        this.profileStorage = config.profiles;
        this.activeProfile = config.active;
    }

    getCardSize() { return 6; }

    setCurrentProfile(profile) {
        this.currentProfileData = profile;
        this.numSetpoints = this.currentProfileData.setpoints.length;
        this.durationSeconds = this.currentProfileData.setpoints[this.currentProfileData.setpoints.length - 1].time;
        this.profileIsActive = this.currentProfileIsActive();
    }

    generateSetpoints(count) {
        const result = [];
        for (let i = 0; i < count; i++) {
            const timePerInterval = this.durationSeconds / (count - 1);
            const timeAtIndex = timePerInterval * i;
            result.push({ time: Number(timeAtIndex.toFixed(1)), temp: 62 + (i * 18) / (count - 1) });
        }
        return result;
    }

    secondsToTimeString(seconds) {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        const secs = Math.floor(seconds % 60);
        return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;
    }

    getTimeLabel(timeInSeconds) { const wrapped = timeInSeconds % 86400; return this.secondsToTimeString(wrapped); }

    clamp(v, a, b) { return Math.min(Math.max(v, a), b); }

    mapY(temp) {
        const fraction = (temp - this.minTemp) / (this.maxTemp - this.minTemp);
        return this.margin.top + this.contentHeight - fraction * this.contentHeight;
    }

    mapX(timeInSeconds) {
        const fraction = timeInSeconds / this.durationSeconds;
        return this.margin.left + fraction * this.contentWidth;
    }

    renderGraph() {
        // Rebuild svg
        this.svg.innerHTML = '';
        this.graphedPoints.length = 0;

        // grid + labels
        const svgns = 'http://www.w3.org/2000/svg';
        const grid = document.createElementNS(svgns, 'g');
        grid.setAttribute('stroke', '#d9e1ff');
        grid.setAttribute('stroke-width', '1');
        grid.setAttribute('opacity', '0.72');

        for (let i = 0; i <= 4; i++) {
            const y = this.margin.top + (this.contentHeight * i) / 4;
            const line = document.createElementNS(svgns, 'line');
            line.setAttribute('x1', this.margin.left);
            line.setAttribute('x2', this.width - this.margin.right);
            line.setAttribute('y1', y);
            line.setAttribute('y2', y);
            if (i === 4) { line.setAttribute('stroke', this.accentColor); line.setAttribute('stroke-width', '1.5'); }
            grid.appendChild(line);

            const label = document.createElementNS(svgns, 'text');
            label.setAttribute('x', this.margin.left - 12);
            label.setAttribute('y', y + 4);
            label.setAttribute('text-anchor', 'end');
            label.setAttribute('fill', this.textColor);
            label.setAttribute('font-size', '14');
            label.textContent = Math.round(this.maxTemp - ((this.maxTemp - this.minTemp) * i) / 4);
            this.svg.appendChild(label);
        }
        this.svg.appendChild(grid);

        // axes
        const axis = document.createElementNS(svgns, 'g');
        const yAxis = document.createElementNS(svgns, 'line');
        yAxis.setAttribute('x1', this.margin.left); yAxis.setAttribute('y1', this.margin.top);
        yAxis.setAttribute('x2', this.margin.left); yAxis.setAttribute('y2', this.height - this.margin.bottom);
        yAxis.setAttribute('stroke', this.accentColor); yAxis.setAttribute('stroke-width', '1.8');
        axis.appendChild(yAxis);
        const xAxis = document.createElementNS(svgns, 'line');
        xAxis.setAttribute('x1', this.margin.left); xAxis.setAttribute('y1', this.height - this.margin.bottom);
        xAxis.setAttribute('x2', this.width - this.margin.right); xAxis.setAttribute('y2', this.height - this.margin.bottom);
        xAxis.setAttribute('stroke', this.accentColor); xAxis.setAttribute('stroke-width', '1.8');
        axis.appendChild(xAxis);
        this.svg.appendChild(axis);

        // path
        const line = document.createElementNS(svgns, 'path');
        const pointsPath = this.currentProfileData.setpoints.map((pt, idx) => `${idx === 0 ? 'M' : 'L'} ${this.mapX(pt.time)} ${this.mapY(pt.temp)}`).join(' ');
        line.setAttribute('d', pointsPath);
        line.setAttribute('fill', 'none'); line.setAttribute('stroke', '#6f83ff'); line.setAttribute('stroke-width', '4');
        line.setAttribute('stroke-linecap', 'round'); line.setAttribute('stroke-linejoin', 'round'); line.setAttribute('opacity', '0.95');
        this.svg.appendChild(line);

        // points
        this.currentProfileData.setpoints.forEach((pt, index) => {
            const x = this.mapX(pt.time);
            const y = this.mapY(pt.temp);
            const isLocked = index === 0 || index === this.currentProfileData.setpoints.length - 1;

            const hit = document.createElementNS(svgns, 'circle');
            hit.setAttribute('cx', x); hit.setAttribute('cy', y); hit.setAttribute('r', 18); hit.setAttribute('fill', 'transparent'); hit.setAttribute('data-index', index);
            hit.setAttribute('cursor', 'grab');
            this.svg.appendChild(hit);

            const marker = document.createElementNS(svgns, 'circle');
            marker.setAttribute('cx', x); marker.setAttribute('cy', y); marker.setAttribute('r', 12);
            marker.setAttribute('fill', this.activeIndex === index ? '#ff9c3b' : '#fff');
            marker.setAttribute('stroke', this.activeIndex === index ? '#ff7e00' : '#6f83ff');
            marker.setAttribute('stroke-width', this.activeIndex === index ? '4' : '3'); marker.setAttribute('data-index', index);
            marker.setAttribute('cursor', 'grab');

            this.svg.appendChild(marker);

            if (isLocked) {
                const lockIcon = document.createElementNS(svgns, 'circle');
                lockIcon.setAttribute('cx', x); lockIcon.setAttribute('cy', y); lockIcon.setAttribute('r', 3); lockIcon.setAttribute('fill', '#6f83ff');
                this.svg.appendChild(lockIcon);
            }

            const label = document.createElementNS(svgns, 'text');
            label.setAttribute('x', x); label.setAttribute('y', y - 20); label.setAttribute('text-anchor', 'middle'); label.setAttribute('fill', this.secondaryTextColor); label.setAttribute('font-size', '14'); label.setAttribute('font-weight', '700');
            label.textContent = `${Number(Math.round(parseFloat(pt.temp + 'e' + 1)) + 'e-' + 1)}`; // to fixed but actually, cause float
            this.svg.appendChild(label);

            const name = document.createElementNS(svgns, 'text');
            name.setAttribute('x', x); name.setAttribute('y', this.height - this.margin.bottom + 24); name.setAttribute('text-anchor', 'middle'); name.setAttribute('fill', this.textColor); name.setAttribute('font-size', '13');
            name.textContent = this.getTimeLabel(pt.time);
            this.svg.appendChild(name);

            this.graphedPoints.push({ x, y, index, isLocked, time: pt.time, temp: pt.temp });
        });

        this.renderInfo();
    }

    renderInfo() {
        this.info.innerHTML = '';
        this.currentProfileData.setpoints.forEach((pt, index) => {
            const chip = document.createElement('div');
            chip.className = 'setpoint-chip' + (this.activeIndex === index ? ' active' : '');
            const isLocked = index === 0 || index === this.currentProfileData.setpoints.length - 1;

            const hours = Math.floor(pt.time / 3600);
            const minutes = Math.floor((pt.time % 3600) / 60);
            const seconds = Math.floor(pt.time % 60);

            let timeContent = '';
            if (isLocked) {
                timeContent = `<div class="label">${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}</div>`;
            } else {
                timeContent = `
          <div class="time-editor">
            <input class="time-hours" min="0" max="23" value="${String(hours).padStart(2, '0')}" data-index="${index}" />
            <span class="time-separator">:</span>
            <input class="time-minutes" min="0" max="59" value="${String(minutes).padStart(2, '0')}" data-index="${index}" />
            <span class="time-separator">:</span>
            <input class="time-seconds" min="0" max="59" value="${String(seconds).padStart(2, '0')}" data-index="${index}" />
          </div>
        `;
            }

            chip.innerHTML = `<div class="value">${pt.temp.toFixed(1)}°</div>${timeContent}`;
            this.info.appendChild(chip);
        });

        // attach listeners
        this.shadowRoot.querySelectorAll('.time-hours, .time-minutes, .time-seconds').forEach(input => {
            input.addEventListener('change', (e) => {
                const index = parseInt(e.target.dataset.index);
                const hoursInput = this.shadowRoot.querySelector(`.time-hours[data-index="${index}"]`);
                const minutesInput = this.shadowRoot.querySelector(`.time-minutes[data-index="${index}"]`);
                const secondsInput = this.shadowRoot.querySelector(`.time-seconds[data-index="${index}"]`);
                const hours = parseInt(hoursInput?.value) || 0;
                const minutes = parseInt(minutesInput?.value) || 0;
                const seconds = parseInt(secondsInput?.value) || 0;
                const newTime = hours * 3600 + minutes * 60 + seconds;
                const minTime = this.currentProfileData.setpoints[index - 1]?.time || 0;
                const maxTime = this.currentProfileData.setpoints[index + 1]?.time || this.durationSeconds;
                this.currentProfileData.setpoints[index].time = Number(this.clamp(newTime, minTime, maxTime).toFixed(0));
                this.renderGraph();
            });
        });
    }

    async saveToEntity() {
        if (!this.profileStorage || !this.hass.states[this.profileStorage]) return; // No entity configured
        try {
            if (this.currentProfileData.profileName !== this.startingProfileName) {
                let profileIndex = this.allProfiles.findIndex(p => p.profileName === this.currentProfileData.profileName);
                if (profileIndex === -1) {
                    this.allProfiles.push(this.currentProfileData);
                } else {
                    this.allProfiles[profileIndex] = { ...this.currentProfileData };
                }
            }

            // const profileJson = JSON.stringify(this.allProfiles);
            // console.log('[SetpointGraphCard] Saved profile to entity:', profileJson, this.config);

            let attribute = { ...this.hass.states[this.profileStorage].attributes };
            attribute.stateAttribute = this.allProfiles;
            console.log('[SetpointGraphCard] Saved profile to entity:', attribute, attribute.stateAttribute);

            // sensor.store_temp_profiles
            await this.hass.callApi('POST', `states/${this.profileStorage}`,
                { state: "stateAttribute", attributes: attribute });

        } catch (e) {
            console.error('[SetpointGraphCard] Failed to save to entity:', e);
        }
        this.loadProfiles();
    }

    async activateProfile() {
        if (!this.activeProfile || !this.hass.states[this.activeProfile]) return; // No entity configured
        try {

            let attribute = { ...this.hass.states[this.activeProfile].attributes };
            attribute.stateAttribute = this.currentProfileData;
            console.log('[SetpointGraphCard] set profile active:', attribute.stateAttribute);

            // sensor.store_temp_profiles
            await this.hass.callApi('POST', `states/${this.activeProfile}`,
                { state: "stateAttribute", attributes: attribute });
            this.profileIsActive = true;
        } catch (e) {
            console.error('[SetpointGraphCard] Failed to activate profile:', e);
        }
    }

    floatEquals(a, b, delta) {
        return Math.abs(a - b) <= delta;
    }

    currentProfileIsActive() {
        if (!this.activeProfile || !this.hass.states[this.activeProfile]) return false;
        let activeProfileData = this.hass.states[this.activeProfile].attributes.stateAttribute;
        if (!activeProfileData) {
            return false;
        }
        if (activeProfileData.setpoints.length !== this.currentProfileData.setpoints.length) {
            return false;
        }
        for (let i = 0; i < activeProfileData.setpoints.length; i++) {
            if (!this.floatEquals(activeProfileData.setpoints[i].time, this.currentProfileData.setpoints[i].time, 0.5) ||
                !this.floatEquals(activeProfileData.setpoints[i].temp, this.currentProfileData.setpoints[i].temp, 0.5)) {
                return false;
            }
        }
        return true;

    }

    loadProfiles() {
        if (!this.profileStorage || !this.hass.states[this.profileStorage]) return; // No entity configured
        let entityFromHass = this.hass.states[this.profileStorage];

        console.log("Loading profiles from entity:", entityFromHass.attributes.stateAttribute, entityFromHass);

        this.allProfiles = entityFromHass.attributes.stateAttribute;
    }

    async DeleteProfile() {
        if (!this.profileStorage || !this.hass.states[this.profileStorage]) return; // No entity configured
        let profileIndex = this.allProfiles.findIndex(p => p.profileName === this.currentProfileData.profileName);
        if (profileIndex !== -1) {
            this.allProfiles.splice(profileIndex, 1);
            this.setCurrentProfile(this.defaultProfile());

            this.saveToEntity();
            this.renderGraph();
        }
    }

    onProfileNameChange(e) {
        const newName = this.shadowRoot.getElementById('newProfileName');
        console.log("Profile name changed to:", newName, e.target.value, e);
        this.currentProfileData.profileName = e.target.value;
        this.renderGraph();
    }

    profileSelected(e) {
        const selectedProfileName = e.target.value;
        const selectedProfile = this.allProfiles.find(p => p.profileName === selectedProfileName);
        if (selectedProfile) {
            this.setCurrentProfile({ ...selectedProfile });
        } else {
            this.setCurrentProfile(this.defaultProfile());
        }
        this.renderGraph();
    }

    getMousePosition(event) {
        const rect = this.svg.getBoundingClientRect();
        const viewBox = this.svg.viewBox.baseVal;
        const scaleX = viewBox.width / rect.width;
        const scaleY = viewBox.height / rect.height;
        return { x: (event.clientX - rect.left) * scaleX, y: (event.clientY - rect.top) * scaleY };
    }

    findPointAt(position) {
        return this.graphedPoints.find(({ x, y }) => {
            const dx = position.x - x; const dy = position.y - y; return Math.sqrt(dx * dx + dy * dy) < 20;
        });
    }

    updatePoint(index, x, y) {
        const point = this.currentProfileData.setpoints[index];
        const isLocked = index === 0 || index === this.currentProfileData.setpoints.length - 1;
        const limitedY = this.clamp(y, this.margin.top, this.height - this.margin.bottom);
        const temp = this.minTemp + ((this.height - this.margin.bottom - limitedY) / this.contentHeight) * (this.maxTemp - this.minTemp);
        point.temp = Number(temp.toFixed(1));

        if (!isLocked) {
            const limitedX = this.clamp(x, this.margin.left, this.width - this.margin.right);
            const newTime = ((limitedX - this.margin.left) / this.contentWidth) * this.durationSeconds;
            const minTime = this.currentProfileData.setpoints[index - 1]?.time || 0;
            const maxTime = this.currentProfileData.setpoints[index + 1]?.time || this.durationSeconds;
            point.time = Number(this.clamp(newTime, minTime, maxTime).toFixed(0));
        }
        this.profileIsActive = this.currentProfileIsActive();
        this.renderGraph();
    }

    // --- Event handlers wired to template or lifecycle ---
    onNumSetpointsChange = (e) => {
        const val = parseInt(e.target.value) || 5;
        let oldNumSetpoints = this.numSetpoints;
        this.numSetpoints = this.clamp(val, 2, 12);
        // if unchanged, nothing to do
        if (this.numSetpoints === oldNumSetpoints) return;

        const oldPoints = this.currentProfileData.setpoints;

        console.log("Resampling setpoints from", oldPoints.length, oldNumSetpoints, "to", this.numSetpoints);
        // Resample helper: linear interpolation along the existing setpoints
        const resample = (srcPoints, targetCount) => {
            const out = [];
            const srcCount = srcPoints.length;
            for (let i = 0; i < targetCount; i++) {
                const alpha = (targetCount === 1) ? 0 : (i / (targetCount - 1)) * (srcCount - 1);
                const lo = Math.floor(alpha);
                const hi = Math.min(srcCount - 1, Math.ceil(alpha));
                const t = alpha - lo;
                const a = srcPoints[lo];
                const b = srcPoints[hi];
                if (!b || lo === hi) {
                    out.push({ time: Math.round(a.time), temp: Number(a.temp.toFixed(1)) });
                } else {
                    const time = Math.round(a.time + (b.time - a.time) * t);
                    const temp = Number((a.temp + (b.temp - a.temp) * t).toFixed(1));
                    out.push({ time, temp });
                }
            }
            return out;
        };


        // Create new setpoints by resampling the current profile
        const newPoints = resample(oldPoints, this.numSetpoints);

        // Ensure first and last times align with 0 and durationSeconds
        newPoints[0].time = 0;
        newPoints[newPoints.length - 1].time = this.durationSeconds;

        this.profileIsActive = false;
        this.currentProfileData.setpoints = newPoints; this.activeIndex = null; this.renderGraph();
    }

    onDurationChange = (e) => {
        const hoursEl = this.shadowRoot.getElementById('durationHours');
        const minutesEl = this.shadowRoot.getElementById('durationMinutes');
        const hours = parseInt(hoursEl.value) || 0; const minutes = parseInt(minutesEl.value) || 0;
        let newDurationSeconds = hours * 3600 + minutes * 60;

        if (newDurationSeconds < 60) {
            newDurationSeconds = 60; // Minimum duration of 1 minute
            hoursEl.value = "0"; minutesEl.value = "1";
        }
        const oldDurationSeconds = this.durationSeconds;

        // Scale existing setpoints proportionally to fit new duration
        const scale = newDurationSeconds / oldDurationSeconds;
        this.currentProfileData.setpoints.forEach((point, index) => {
            point.time = Math.round(point.time * scale);
        });

        // Ensure anchored times: first at 0, last at new duration
        this.currentProfileData.setpoints[0].time = 0;
        this.currentProfileData.setpoints[this.currentProfileData.setpoints.length - 1].time = newDurationSeconds;

        this.profileIsActive = this.currentProfileIsActive();
        this.durationSeconds = newDurationSeconds; this.activeIndex = null; this.renderGraph();
    }


    _onPointerDown = (event) => {
        event.preventDefault();
        const position = this.getMousePosition(event);
        const targetPoint = this.findPointAt(position);
        if (!targetPoint) { this.activeIndex = null; this.renderGraph(); return; }
        this.activeIndex = targetPoint.index; this.svg.setPointerCapture(event.pointerId); this.updatePoint(this.activeIndex, position.x, position.y);
    }

    _onPointerMove = (event) => {
        if (this.activeIndex === null) return; event.preventDefault(); const position = this.getMousePosition(event); this.updatePoint(this.activeIndex, position.x, position.y);
    }

    _onPointerUp = (event) => { if (this.activeIndex !== null) this.svg.releasePointerCapture(event.pointerId); this.activeIndex = null; this.renderGraph(); }

    _onPointerLeave = () => { this.activeIndex = null; this.renderGraph(); }
}

customElements.define('setpoint-graph-card', SetpointGraphCard);