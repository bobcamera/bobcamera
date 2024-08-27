import * as fromRoot from '../../state';

export const NIGHT_MODE_THEME = 'BLACK-THEME';

export type Language = 'en' | 'sk' | 'de' | 'fr' | 'es' | 'pt-br' | 'he' | 'ar';

export interface RosConnectionModel {
  url: string;
  port: number;
}

export interface SettingsState {
  language: string;
  theme: string;
  autoNightMode: boolean;
  nightTheme: string;
  stickyHeader: boolean;
  pageAnimations: boolean;
  pageAnimationsDisabled: boolean;
  elementsAnimations: boolean;
  hour: number;
  rosModel: RosConnectionModel;
}

/*export interface State extends fromRoot.State {
  settings: SettingsState;
}*/
