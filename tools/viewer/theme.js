// Classic script in <head> so the saved or system theme applies before first paint.
(() => {
  let saved = null;
  try { saved = localStorage.getItem('maxscope.theme'); } catch { /* Storage may be disabled. */ }
  const system = matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  document.documentElement.dataset.theme = saved === 'dark' || saved === 'light' ? saved : system;
})();
