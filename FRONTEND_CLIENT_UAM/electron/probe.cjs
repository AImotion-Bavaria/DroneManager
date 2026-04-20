console.log('versions.electron:', process.versions.electron);
console.log('process.type:', process.type);
const lb = process._linkedBinding;
if (lb) {
  // Try known electron internal binding names
  ['electron_browser_app','electron_common_v8_util','electron_browser_browser_window'].forEach(name => {
    try { const b = lb(name); console.log(name+':', typeof b, Object.keys(b||{}).slice(0,5)); }
    catch(e) { console.log(name+': err -', e.message.slice(0,50)); }
  });
}
process.exit(0);
