document.addEventListener("DOMContentLoaded", function () {
  const globaltoc = document.querySelector(".globaltoc");
  if (!globaltoc) return;

  const captions = globaltoc.querySelectorAll("p.caption");
  captions.forEach((caption, index) => {
    const list = caption.nextElementSibling;
    if (!list || list.tagName.toLowerCase() !== "ul") return;

    // Check if this section contains the active document
    const hasActiveItem = list.querySelector("li.current") !== null;

    // Create chevron icon
    const chevron = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    chevron.setAttribute("class", "caption-chevron");
    chevron.setAttribute("viewBox", "0 0 24 24");
    chevron.innerHTML = '<polyline points="6 9 12 15 18 9"></polyline>';
    caption.appendChild(chevron);

    const storageKey = "rig_wiki_collapsed_" + (caption.textContent || index).trim();
    const isSavedCollapsed = localStorage.getItem(storageKey) === "true";

    // Auto-expand if active, otherwise check stored state
    if (!hasActiveItem && isSavedCollapsed) {
      caption.classList.add("is-collapsed");
      list.classList.add("is-collapsed");
    }

    caption.addEventListener("click", function () {
      const willCollapse = !caption.classList.contains("is-collapsed");
      caption.classList.toggle("is-collapsed", willCollapse);
      list.classList.toggle("is-collapsed", willCollapse);
      localStorage.setItem(storageKey, willCollapse ? "true" : "false");
    });
  });
});
