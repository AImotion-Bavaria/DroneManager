const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  onDMStatus: (callback) => {
    ipcRenderer.on('dm-status', (_event, status) => callback(status));
  },
  removeDMStatusListener: () => {
    ipcRenderer.removeAllListeners('dm-status');
  },
});
