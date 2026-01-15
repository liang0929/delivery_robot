import { BrowserRouter, Routes, Route } from 'react-router-dom';
import { Layout } from './components/layout/Layout';
import { RemoteControl } from './pages/RemoteControl';
import { SlamMapping } from './pages/SlamMapping';
import { Navigation } from './pages/Navigation';
import { SystemStatus } from './pages/SystemStatus';

function App() {
  return (
    <BrowserRouter>
      <Layout>
        <Routes>
          <Route path="/" element={<RemoteControl />} />
          <Route path="/slam" element={<SlamMapping />} />
          <Route path="/navigation" element={<Navigation />} />
          <Route path="/status" element={<SystemStatus />} />
        </Routes>
      </Layout>
    </BrowserRouter>
  );
}

export default App;
