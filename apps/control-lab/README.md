# Control Lab dashboard

This directory is a standalone, static Vercel project for the interactive water-tank control engineering dashboard.

## Live deployment

Production: <https://control-lab-alpha.vercel.app>

The local directory is linked to the `control-lab` project in Samuel's Vercel account. The `.vercel/` link metadata is intentionally excluded from Git so repository imports can select their own Vercel account and project.

## Project layout

- `dist/` contains the deployable P, PI, and coupled two-tank pages.
- `scripts/validate-site.mjs` verifies page entry points, local assets, DOM IDs, and JavaScript syntax.
- `vercel.json` tells Vercel to run the validation build and publish `dist/`.
- Vercel clean URLs expose `/pi-control` and `/two-tank`; existing `.html` links are redirected automatically.
- `.openai/hosting.json` is retained only for the existing ChatGPT Site and is excluded from Vercel uploads.

## Validate locally

Node.js 20 or newer is required. The project has no runtime dependencies.

```bash
npm run build
```

## Deploy with Vercel CLI

From this directory:

```bash
npx vercel
```

After checking the preview deployment, publish it to production:

```bash
npx vercel --prod
```

Vercel will ask you to sign in and choose an account/project the first time. No environment variables are required.

## Deploy from a Git repository

Import the repository in Vercel. If this full course repository is used, set **Root Directory** to `apps/control-lab`. The configuration committed here supplies:

- Build command: `npm run build`
- Output directory: `dist`
- Framework preset: Other

Every push to the connected production branch can then create a new deployment.
