import { kml } from "@tmcw/togeojson";

function parseKmlText(kmlText) {
  const parser = new DOMParser();
  const xml = parser.parseFromString(kmlText, "application/xml");
  return kml(xml);
}
export default parseKmlText;
