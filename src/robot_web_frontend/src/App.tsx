import { BrowserRouter, Routes, Route } from 'react-router-dom';

// Layouts
import { WaiterLayout } from './components/layout/WaiterLayout';
import { AdminLayout } from './components/layout/AdminLayout';

// Pages
import { RoleSelect } from './pages/RoleSelect';
import { WaiterDashboard } from './pages/waiter/WaiterDashboard';
import { AdminDashboard } from './pages/admin/AdminDashboard';
import { TableManagement } from './pages/admin/TableManagement';
import { RemoteControl } from './pages/RemoteControl';
import { SlamMapping } from './pages/SlamMapping';
import { SystemStatus } from './pages/SystemStatus';

function App() {
  return (
    <BrowserRouter>
      <Routes>
        {/* Role Selection (Home) */}
        <Route path="/" element={<RoleSelect />} />

        {/* Waiter Routes */}
        <Route
          path="/waiter"
          element={
            <WaiterLayout>
              <WaiterDashboard />
            </WaiterLayout>
          }
        />

        {/* Admin Routes */}
        <Route
          path="/admin"
          element={
            <AdminLayout>
              <AdminDashboard />
            </AdminLayout>
          }
        />
        <Route
          path="/admin/tables"
          element={
            <AdminLayout>
              <TableManagement />
            </AdminLayout>
          }
        />
        <Route
          path="/admin/slam"
          element={
            <AdminLayout>
              <SlamMapping />
            </AdminLayout>
          }
        />
        <Route
          path="/admin/remote"
          element={
            <AdminLayout>
              <RemoteControl />
            </AdminLayout>
          }
        />
        <Route
          path="/admin/status"
          element={
            <AdminLayout>
              <SystemStatus />
            </AdminLayout>
          }
        />
      </Routes>
    </BrowserRouter>
  );
}

export default App;
