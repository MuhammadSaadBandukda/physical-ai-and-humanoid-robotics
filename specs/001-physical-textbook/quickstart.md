# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

This guide provides instructions for setting up your development environment for the "Physical AI & Humanoid Robotics Textbook" project.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

*   **Git**: For version control.
*   **Node.js (LTS) & npm**: For the Docusaurus frontend.
*   **Python 3.10+**: For the FastAPI backend.
*   **Poetry**: Python dependency management.
    *   *Installation:* `pip install poetry`
*   **Docker Desktop**: Required for the simulated (amd64) and physical (arm64) ROS2 environments.

### PowerShell Execution Policy (Windows Users)

If you encounter errors like "running scripts is disabled on this system" when using `npx` or `npm`, you may need to adjust your PowerShell Execution Policy. Open PowerShell **as an Administrator** and run:

```powershell
Set-ExecutionPolicy RemoteSigned -Scope CurrentUser
```

## 2. Project Setup

1.  **Clone the repository:**
    ```bash
    git clone [repository-url]
    cd physical-ai-and-humanoid-robotics
    ```

2.  **Install Frontend Dependencies:**
    Navigate to the `frontend` directory and install Node.js dependencies.
    ```bash
    cd frontend
    npm install
    ```

3.  **Install Backend Dependencies (Poetry Crash Course):**
    The backend uses `Poetry` for dependency management.

    *   **Initialize Poetry:** If `poetry.lock` doesn't exist or is outdated, you can run:
        ```bash
        cd backend
        poetry lock --no-update # Generates poetry.lock without updating dependencies
        poetry install         # Installs dependencies from poetry.lock
        ```
        *Note*: If you are on a bleeding-edge Python version (e.g., Python 3.14 on Windows), you might encounter issues with certain packages (like `numpy`) not having pre-built wheels. In this case, it is highly recommended to use the Docker environment for backend development.

    *   **Running commands with Poetry:**
        To run any Python script or command within the Poetry virtual environment, prefix it with `poetry run`.
        Example: `poetry run pytest` or `poetry run python your_script.py`

## 3. Running the Development Servers

### Frontend (Docusaurus)

To start the Docusaurus development server:

```bash
cd frontend
npm start
```
This will open the textbook site in your browser at `http://localhost:3000`.

### Backend (FastAPI - Deferred)

*Currently deferred due to strategic pivot. Will be covered in Phase 2.*

## 4. Docker Environments (Currently Deferred)

*Instructions for building and running Docker containers are currently deferred to Phase 2.*
