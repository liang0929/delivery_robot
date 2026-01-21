/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_ROBOT_IP?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
