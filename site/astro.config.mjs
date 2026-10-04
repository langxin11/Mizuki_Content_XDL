import { defineConfig } from "astro/config";
import shirones from "shirones";
export default defineConfig({ integrations: [shirones({ pagefind: false,
  excludeRoutes: ["/friends", "/moments", "/anime", "/compass", "/albums", "/albums/[id]", "/skills", "/projects", "/devices", "/games", "/timeline", "/series", "/series/[slug]", "/llms.txt", "/llms-full.txt", "/robots.txt"]
})] });
