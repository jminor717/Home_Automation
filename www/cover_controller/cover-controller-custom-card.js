import {
	LitElement,
	html,
	css,
} from "https://unpkg.com/lit-element@2.0.1/lit-element.js?module";

class CoverControllerCustomCard extends LitElement {
	static get properties() {
		return {
			hass: {},
			config: {},
		};
	}

	static get styles() {
		return css`
			.card { padding: 16px; }
			h2 { margin: 0 0 12px; font-size: 1.2rem; }
			.cover {
				display: grid;
				grid-template-columns: minmax(0, 1fr) auto;
				gap: 12px;
				align-items: center;
				padding: 12px 0;
				border-bottom: 1px solid var(--divider-color, #ddd);
			}
			.cover:last-child { border-bottom: 0; }
			.name { font-weight: 600; overflow-wrap: anywhere; }
			.state { margin-top: 4px; color: var(--secondary-text-color, #666); font-size: 0.9rem; text-transform: capitalize; }
			.controls { display: flex; gap: 6px; flex-wrap: wrap; justify-content: flex-end; }
			button {
				min-height: 36px;
				padding: 6px 10px;
				border: 1px solid var(--divider-color, #bbb);
				border-radius: 4px;
				background: var(--card-background-color, white);
				color: var(--primary-text-color, #222);
				font: inherit;
				cursor: pointer;
			}
			button:disabled { opacity: 0.5; cursor: default; }
			.position {
				grid-column: 1 / -1;
				display: grid;
				grid-template-columns: auto minmax(80px, 1fr) 3em;
				align-items: center;
				gap: 10px;
				color: var(--secondary-text-color, #666);
				font-size: 0.9rem;
			}
			input[type="range"] { width: 100%; accent-color: var(--primary-color, #03a9f4); }
			.empty { color: var(--secondary-text-color, #666); }
		`;
	}

	setConfig(config) { this.config = config || {}; }

	get covers() {
		if (!this.hass || !this.hass.states) return [];
		return Object.values(this.hass.states).filter((entity) =>
			entity.entity_id.startsWith("cover.") && entity.entity_id.endsWith("_erv")
		);
	}

	render() {
		const covers = this.covers;
		const title = this.config?.title || "ERV Covers";

		return html`
			<ha-card>
				<div class="card">
					<h2>${title}</h2>
					${covers.length
						? covers.map((cover) => this.renderCover(cover))
						: html`<div class="empty">No cover entities ending in _erv found.</div>`}
				</div>
			</ha-card>
		`;
	}

	renderCover(cover) {
		const name = cover.attributes.friendly_name || cover.entity_id;
		const supportsPosition = typeof cover.attributes.current_position === "number" || ((cover.attributes.supported_features || 0) & 4) !== 0;
		const position = cover.attributes.current_position;

		return html`
			<section class="cover">
				<div>
					<div class="name">${name}</div>
					<div class="state">${cover.state}</div>
				</div>
				<div class="controls">
					<button
						aria-label="Open ${name}"
						?disabled=${cover.state === "open" || cover.state === "opening"}
						@click=${() => this.callCoverService("open_cover", cover.entity_id)}
					>Open</button>
					<button
						aria-label="Stop ${name}"
						?disabled=${cover.state !== "opening" && cover.state !== "closing"}
						@click=${() => this.callCoverService("stop_cover", cover.entity_id)}
					>Stop</button>
					<button
						aria-label="Close ${name}"
						?disabled=${cover.state === "closed" || cover.state === "closing"}
						@click=${() => this.callCoverService("close_cover", cover.entity_id)}
					>Close</button>
				</div>
				${supportsPosition
					? html`
						<label class="position">
							<span>Position</span>
							<input
								type="range"
								min="0"
								max="100"
								.value=${String(position ?? 0)}
								aria-label="Position of ${name}"
								@change=${(event) => this.setCoverPosition(cover.entity_id, event.target.value)}
							/>
							<span>${position ?? "--"}%</span>
						</label>
					`
					: ""}
			</section>
		`;
	}

	callCoverService(service, entityId) {
		this.hass.callService("cover", service, { entity_id: entityId });
	}

	setCoverPosition(entityId, position) {
		this.hass.callService("cover", "set_cover_position", {
			entity_id: entityId,
			position: Number(position),
		});
	}

	getCardSize() {
		return Math.max(2, this.covers.length * 2 + 1);
	}
}

customElements.define("cover-controller-custom-card", CoverControllerCustomCard);






